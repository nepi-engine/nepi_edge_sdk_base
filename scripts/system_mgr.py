#!/usr/bin/env python
import os
import shutil
import random  # Only necessary for simulation of temperature data
from collections import deque
import re
from datetime import datetime

import subprocess

import rospy

from std_msgs.msg import String, Empty, Float32
from nepi_ros_interfaces.msg import SystemStatus, SystemDefs, WarningFlags, StampedString, SaveData
from nepi_ros_interfaces.srv import SystemDefsQuery, SystemDefsQueryResponse, OpEnvironmentQuery, OpEnvironmentQueryResponse, \
                             SystemSoftwareStatusQuery, SystemSoftwareStatusQueryResponse

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
import nepi_edge_sdk_base.nepi_software_update_utils as sw_update_utils

BYTES_PER_MEGABYTE = 2**20


class SystemMgrNode():
    NODE_NAME = "system_mgr"

    STATUS_PERIOD = 1.0  # TODO: Configurable update period?

    DISK_FULL_MARGIN_MB = 250  # MB TODO: Configurable?

    SYS_ENV_PATH = "/opt/nepi/sys_env.bash"
    FW_VERSION_PATH = "/opt/nepi/ros/etc/fw_version.txt"

    status_msg = SystemStatus()
    system_defs_msg = SystemDefs()

    data_mountpoint = "/home/numurus_user/data"

    # disk_usage_deque = deque(maxlen=10)
    # Shorter period for more responsive updates
    disk_usage_deque = deque(maxlen=3)

    first_stage_rootfs_device = "/dev/mmcblk0p1"
    new_img_staging_device = "/dev/mmcblk1p3"
    new_img_staging_device_removable = False
    usb_device = "/dev/sda" 
    sd_card_device = "/dev/mmcblk1p"
    emmc_device = "/dev/mmcblk0p"
    auto_switch_rootfs_on_new_img_install = True
    sw_update_progress = ""

    installing_new_image = False
    archiving_inactive_image = False

    def add_info_string(self, string, level):
        self.status_msg.info_strings.append(StampedString(
            timestamp=rospy.get_rostime(), payload=string, priority=level))

    def get_device_sn(self):
        with open(self.SYS_ENV_PATH, "r") as f:
            for line in f:
                if line.startswith("export DEVICE_SN="):
                    return line.split('=')[1].rstrip()
        return "undefined"

    def get_fw_rev(self):
        if (os.path.exists(self.FW_VERSION_PATH) and (os.path.getsize(self.FW_VERSION_PATH) > 0)):
            with open(self.FW_VERSION_PATH, "r") as f:
                return f.readline().strip()
        return "UNSPECIFIED"

    def update_temperatures(self):
        # Get the current temperatures

        # TODO: Should the temperature sensor or the entire subproc. cmd line be configurable?
        temp_string_mdegC = subprocess.check_output(
            ["cat", "/sys/class/thermal/thermal_zone0/temp"])
        self.status_msg.temperatures[0] = float(temp_string_mdegC) / 1000.0

        # Check for temperature warnings and do thermal throttling
        throttle_ratio_min = 1.0
        for i, t in enumerate(self.status_msg.temperatures):
            if (t > self.system_defs_msg.critical_temps[i]):
                # Critical implies high
                self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = True
                self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
                # Make sure to send a user string
                self.add_info_string(
                    WarningFlags.CRITICAL_TEMPERATURE_STRING, StampedString.PRI_HIGH)
                # Set the throttle ratio to 0% globally
                rospy.logerr_throttle(
                    10, "%s: temperature = %f", WarningFlags.CRITICAL_TEMPERATURE_STRING, t)
                throttle_ratio_min = 0.0
            else:
                self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = False
                if (t > self.system_defs_msg.warning_temps[i]):
                    self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
                    throttle_ratio_i = 1.0 - ((t - self.system_defs_msg.warning_temps[i]) / (
                        self.system_defs_msg.critical_temps[i] - self.system_defs_msg.warning_temps[i]))
                    rospy.logwarn_throttle(
                        10, "%s: temperature = %f", WarningFlags.HIGH_TEMPERATURE_STRING, t)
                    throttle_ratio_min = min(
                        throttle_ratio_i, throttle_ratio_min)
                else:
                    self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = False
            # If a new thermal throttle ratio was computed, publish it globally
            if (throttle_ratio_min != self.current_throttle_ratio):
                self.throttle_ratio_pub.publish(Float32(throttle_ratio_min))
                self.current_throttle_ratio = throttle_ratio_min
                rospy.logwarn("New thermal rate throttle value: %f%%",
                              self.current_throttle_ratio)

    def update_storage(self):
        # Data partition
        statvfs = os.statvfs(self.data_mountpoint)
        disk_free = float(statvfs.f_frsize) * \
            statvfs.f_bavail / BYTES_PER_MEGABYTE  # In MB
        self.status_msg.disk_usage = self.system_defs_msg.disk_capacity - disk_free

        self.disk_usage_deque.append(self.status_msg.disk_usage)
        self.status_msg.storage_rate = (
            self.disk_usage_deque[-1] - self.disk_usage_deque[0]) / (len(self.disk_usage_deque)*self.STATUS_PERIOD)

        # Check for disk warnings
        if (disk_free < self.DISK_FULL_MARGIN_MB):
            self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = True
            self.add_info_string("Max disk usage exceeded",
                                 StampedString.PRI_HIGH)
            # Force all nodes to stop data saving
            self.save_data_pub.publish(save_continuous=False, save_raw=False)
        else:
            self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = False

    def provide_sw_update_status(self, req):
        resp = SystemSoftwareStatusQueryResponse()
        resp.new_sys_img_staging_device = self.get_device_friendly_name(self.new_img_staging_device)
        resp.new_sys_img_staging_device_free_mb = sw_update_utils.getPartitionFreeByteCount(self.new_img_staging_device) / BYTES_PER_MEGABYTE

        # Don't query anything if we are in the middle of installing a new image
        if self.installing_new_image:
            resp.new_sys_img = 'busy'
            resp.new_sys_img_version = 'busy'
            return resp

        (status, err_string, new_img_file, new_img_version, new_img_filesize) = sw_update_utils.checkForNewImageAvailable(
            self.new_img_staging_device, self.new_img_staging_device_removable)
        if status is False:
            rospy.logwarn("Unable to update software status: " + err_string)
            resp.new_sys_img = 'query failed'
            resp.new_sys_img_version = 'query failed'
            resp.new_sys_img_size_mb = 0
            self.status_msg.sys_img_update_status = 'query failed'
            return resp

        # Update the response
        if new_img_file:
            resp.new_sys_img = new_img_file
            resp.new_sys_img_version = new_img_version
            resp.new_sys_img_size_mb = new_img_filesize / BYTES_PER_MEGABYTE
            if not self.status_msg.sys_img_update_status:
                self.status_msg.sys_img_update_status = "ready to install"
        else:
            resp.new_sys_img = 'none detected'
            resp.new_sys_img_version = 'none detected'
            resp.new_sys_img_size_mb = 0
                
        return resp

    def publish_periodic_status(self, event):
        self.status_msg.sys_time = event.current_real

        # Populate the rest of the message contents
        # Temperature(s)
        self.update_temperatures()

        # Disk usage
        self.update_storage()

        # Now publish it
        self.status_pub.publish(self.status_msg)

        # Always clear info strings after publishing
        del self.status_msg.info_strings[:]

    def set_save_status(self, save_msg):
        self.status_msg.save_all_enabled = save_msg.save_continuous

    def clear_data_folder(self, msg):
        if (self.status_msg.save_all_enabled is True):
            rospy.logwarn(
                "Refusing to clear data folder because data saving is currently enabled")
            return

        rospy.loginfo("Clearing data folder by request")
        for filename in os.listdir(self.data_mountpoint):
            file_path = os.path.join(self.data_mountpoint, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                rospy.logwarn('Failed to delete %s. Reason: %s' %
                              (file_path, e))

    def set_op_environment(self, msg):
        if (msg.data != OpEnvironmentQueryResponse.OP_ENV_AIR) and (msg.data != OpEnvironmentQueryResponse.OP_ENV_WATER):
            rospy.logwarn(
                "Setting environment parameter to a non-standard value: %s", msg.data)
        rospy.set_param("~op_environment", msg.data)

    def set_device_id(self, msg):
        # First, validate the characters in the msg as namespace chars -- blank string is okay here to clear the value
        if (msg.data) and (not self.valid_device_id_re.match(msg.data)):
            rospy.logerr("Invalid device ID: %s", msg.data)
            return

        # Otherwise, overwrite the DEVICE_ID in sys_env.bash
        file_lines = []
        with open(self.SYS_ENV_PATH, "r") as f:
            for line in f:
                if line.startswith("export DEVICE_ID"):
                    file_lines.append("export DEVICE_ID=" + msg.data + '\n')
                else:
                    file_lines.append(line)
        tmp_filename = self.SYS_ENV_PATH + ".tmp"
        with open(tmp_filename, "w") as f:
            for line in file_lines:
                f.write(line)

        # Now overwrite the original file as an autonomous operation
        os.rename(tmp_filename, self.SYS_ENV_PATH)
        rospy.logwarn("Device ID Updated - Requires device reboot")
        self.add_info_string(
            "Device ID updated - Requires device reboot", StampedString.PRI_ELEVATED)

    def handle_system_error_msg(self, msg):
        self.add_info_string(msg.data, StampedString.PRI_HIGH)

    def receive_sw_update_progress(self, progress_val):
        self.status_msg.sys_img_update_progress = progress_val

    def receive_archive_progress(self, progress_val):
        self.status_msg.sys_img_archive_progress = progress_val
    
    def handle_install_new_img(self, msg):
        if self.installing_new_image:
            rospy.logwarn("New image is already being installed")
            return

        decompressed_img_filename = msg.data
        self.status_msg.sys_img_update_status = 'flashing'
        self.installing_new_image = True

        status, err_msg = sw_update_utils.writeImage(self.new_img_staging_device, decompressed_img_filename, self.inactive_rootfs_device, 
                                                     do_slow_transfer=False, progress_cb=self.receive_sw_update_progress)

        # Finished installing
        self.installing_new_image = False
        if status is False:
            rospy.logerr("Failed to flash image: " + err_msg)
            self.status_msg.sys_img_update_status = 'failed'
            return
        else:
            rospy.loginfo("Finished flashing new image to inactive partition")
            self.status_msg.sys_img_update_status = 'complete - needs rootfs switch and reboot'

        # Check and repair the newly written filesystem as necessary
        status, err_msg = sw_update_utils.checkAndRepairPartition(self.inactive_rootfs_device)
        if status is False:
            rospy.logerr("Newly flashed image has irrepairable filesystem issues: ", err_msg)
            self.status_msg.sys_img_update_status = 'failed - fs errors'
            return
        else:
            rospy.loginfo("New image filesystem checked and repaired (as necessary)")

        # Do automatic rootfs switch if so configured
        if self.auto_switch_rootfs_on_new_img_install:
            status, err_msg = sw_update_utils.switchActiveAndInactivePartitions(
                self.first_stage_rootfs_device)
            if status is False:
                rospy.logwarn("Automatic rootfs active/inactive switch failed: " + err_msg)
            else:
                rospy.loginfo("Executed automatic rootfs A/B switch... on next reboot new image will load")
                self.status_msg.sys_img_update_status = 'complete - needs reboot'
    
    def handle_switch_active_inactive_rootfs(self, msg):
        status, err_msg = sw_update_utils.switchActiveAndInactivePartitions(
            self.first_stage_rootfs_device)
        if status is False:
            rospy.logwarn("Failed to switch active/inactive rootfs: " + err_msg)
            return

        rospy.logwarn(
            "Switched active and inactive rootfs. Must reboot system for changes to take effect")

    def handle_archive_inactive_rootfs(self, msg):
        if self.archiving_inactive_image is True:
            rospy.logwarn("Already in the process of archiving image")
            return

        now = datetime.now()
        backup_file_basename = 'nepi_rootfs_archive_' + now.strftime("%Y_%m_%d_%H%M%S") + '.img.raw'
        self.status_msg.sys_img_archive_status = 'archiving'
        self.status_msg.sys_img_archive_filename = backup_file_basename

        # Transfers to USB seem to have trouble with the standard block size, so allow those to proceed at a lower
        # block size
        slow_transfer = True if self.usb_device in self.new_img_staging_device else False
                
        self.archiving_inactive_image = True
        status, err_msg = sw_update_utils.archiveInactiveToStaging(
            self.inactive_rootfs_device, self.new_img_staging_device, backup_file_basename, slow_transfer, progress_cb = self.receive_archive_progress)
        self.archiving_inactive_image = False

        if status is False:
            rospy.logerr("Failed to backup inactive rootfs: " + err_msg)
            self.status_msg.sys_img_archive_status = 'failed'
        else:
            rospy.loginfo("Finished archiving inactive rootfs")
            self.status_msg.sys_img_archive_status = 'archive complete'
    
    def provide_system_defs(self, req):
        return SystemDefsQueryResponse(self.system_defs_msg)

    def provide_op_environment(self, req):
        # Just proxy the param server
        return OpEnvironmentQueryResponse(rospy.get_param("~op_environment", OpEnvironmentQueryResponse.OP_ENV_AIR))

    def get_device_friendly_name(self, devfs_name):
        # Leave space for the partition number
        friendly_name = devfs_name.replace(self.emmc_device, "EMMC Partition ")
        friendly_name = friendly_name.replace(self.usb_device, "USB Partition ")
        friendly_name = friendly_name.replace(self.sd_card_device, "SD/SSD Partition ")
        return friendly_name

    def init_msgs(self):
        self.system_defs_msg.firmware_version = self.get_fw_rev()

        self.system_defs_msg.device_sn = self.get_device_sn()

        # TODO: Determine how many temperature readings we have. On Jetson, for example
        #      there are 8 "thermal zones" in /sys/class/thermal/
        self.system_defs_msg.temperature_sensor_names.append('CPU Zone 0')
        # TODO: Configurable warning/error temperatures
        self.system_defs_msg.warning_temps.append(60.0)
        self.system_defs_msg.critical_temps.append(70.0)

        statvfs = os.statvfs(self.data_mountpoint)
        self.system_defs_msg.disk_capacity = statvfs.f_frsize * statvfs.f_blocks / \
            BYTES_PER_MEGABYTE     # Size of data filesystem in Megabytes

        self.system_defs_msg.first_stage_rootfs_device = self.get_device_friendly_name(self.first_stage_rootfs_device)
        
        # Gather some info about ROOTFS A/B configuration
        (status, err_msg, rootfs_ab_settings_dict) = sw_update_utils.getRootfsABStatus(
            self.first_stage_rootfs_device)
        if status is True:
            self.system_defs_msg.active_rootfs_device = self.get_device_friendly_name(rootfs_ab_settings_dict[
                'active_part_device'])

            self.system_defs_msg.active_rootfs_size_mb = sw_update_utils.getPartitionByteCount(rootfs_ab_settings_dict[
                'active_part_device']) / BYTES_PER_MEGABYTE
            
            self.inactive_rootfs_device = rootfs_ab_settings_dict[
                'inactive_part_device']
            self.system_defs_msg.inactive_rootfs_device = self.get_device_friendly_name(self.inactive_rootfs_device)

            self.system_defs_msg.inactive_rootfs_size_mb = sw_update_utils.getPartitionByteCount(self.inactive_rootfs_device) / BYTES_PER_MEGABYTE
            
            self.system_defs_msg.inactive_rootfs_fw_version = rootfs_ab_settings_dict[
                'inactive_part_fw_version']
            self.system_defs_msg.max_boot_fail_count = rootfs_ab_settings_dict[
                'max_boot_fail_count']
        else:
            rospy.logwarn(
                "Unable to gather ROOTFS A/B system definitions: " + err_msg)
            self.system_defs_msg.active_rootfs_device = "Unknown"
            self.system_defs_msg.inactive_rootfs_device = "Unknown"
            self.inactive_rootfs_device = "Unknown"
            self.system_defs_msg.inactive_rootfs_fw_version = "Unknown"
            self.system_defs_msg.max_boot_fail_count = 0

        for i in self.system_defs_msg.temperature_sensor_names:
            self.status_msg.temperatures.append(0.0)

	    # TODO: Should this be queried somehow e.g., from the param server
        self.status_msg.save_all_enabled = False

    def updateFromParamServer(self):
        op_env = rospy.get_param(
            "~op_environment", OpEnvironmentQueryResponse.OP_ENV_AIR)
        # Publish it to all subscribers (which includes this node) to ensure the parameter is applied
        self.set_op_env_pub.publish(String(op_env))

        self.data_mountpoint = rospy.get_param(
            "~data_mountpoint", self.data_mountpoint)

        self.first_stage_rootfs_device = rospy.get_param(
            "~first_stage_rootfs_device", self.first_stage_rootfs_device
        )

        self.new_img_staging_device = rospy.get_param(
            "~new_img_staging_device", self.new_img_staging_device
        )

        self.new_img_staging_device_removable = rospy.get_param(
            "~new_img_staging_device_removable", self.new_img_staging_device_removable
        )

        self.emmc_device = rospy.get_param(
            "~emmc_device", self.emmc_device
        )

        self.usb_device = rospy.get_param(
            "~usb_device", self.usb_device
        )

        self.sd_card_device = rospy.get_param(
            "~sd_card_device", self.sd_card_device
        )

        self.auto_switch_rootfs_on_new_img_install = rospy.get_param(
            "~auto_switch_rootfs_on_new_img_install", self.auto_switch_rootfs_on_new_img_install
        )
    
    def run(self):
        # Want to update the op_environment (from param server) through the whole system once at
        # start-up, but the only reasonable way to do that is to delay long enough to let all nodes start
        rospy.sleep(3)
        self.updateFromParamServer()

        # Reset the A/B rootfs boot fail counter -- if this node is running, pretty safe bet that we've booted successfully
        # This should be redundant, as we need a non-ROS reset mechanism, too, in case e.g., ROS nodes are delayed waiting
        # for a remote ROS master to start. That could be done in roslaunch.sh or a separate start-up script.
        status, err_msg = sw_update_utils.resetBootFailCounter(
            self.first_stage_rootfs_device)
        if status is False:
            rospy.logerr("Failed to reset boot fail counter: " + err_msg)

        rospy.Timer(rospy.Duration(self.STATUS_PERIOD),
                    self.publish_periodic_status)

        # Call the method to update s/w status once internally to prime the status fields now that we have all the parameters
        # established
        self.provide_sw_update_status(0) # Any argument is fine here as the req. field is unused
        
        rospy.spin()

    def __init__(self):
        rospy.loginfo("Starting " + self.NODE_NAME + "node")
        rospy.init_node(self.NODE_NAME)

        status_period = rospy.Duration(1)  # TODO: Configurable rate?

        # Announce published topics
        self.status_pub = rospy.Publisher(
            'system_status', SystemStatus, queue_size=1)
        self.store_params_pub = rospy.Publisher(
            'store_params', String, queue_size=10)
        self.throttle_ratio_pub = rospy.Publisher(
            'apply_throttle', Float32, queue_size=3)
        self.set_op_env_pub = rospy.Publisher(
            'set_op_environment', String, queue_size=3)
        # For auto-stop of save data; disk full protection
        self.save_data_pub = rospy.Publisher(
            'save_data', SaveData, queue_size=1)

        self.current_throttle_ratio = 1.0

        # Subscribe to topics
        rospy.Subscriber('save_data', SaveData, self.set_save_status)
        rospy.Subscriber('clear_data_folder', Empty, self.clear_data_folder)
        rospy.Subscriber('set_op_environment', String, self.set_op_environment)

        rospy.Subscriber('set_device_id', String,
                         self.set_device_id)  # Public ns

        rospy.Subscriber('submit_system_error_msg', String,
                         self.handle_system_error_msg)

        rospy.Subscriber('install_new_image', String,
                         self.handle_install_new_img, queue_size=1)

        rospy.Subscriber('switch_active_inactive_rootfs', Empty,
                         self.handle_switch_active_inactive_rootfs)

        rospy.Subscriber('archive_inactive_rootfs', Empty, self.handle_archive_inactive_rootfs, queue_size=1)

        # Advertise services
        rospy.Service('system_defs_query', SystemDefsQuery,
                      self.provide_system_defs)
        rospy.Service('op_environment_query', OpEnvironmentQuery,
                      self.provide_op_environment)
        rospy.Service('sw_update_status_query', SystemSoftwareStatusQuery,
                       self.provide_sw_update_status)

        self.save_cfg_if = SaveCfgIF(
            updateParamsCallback=None, paramsModifiedCallback=self.updateFromParamServer)

        # Need to get the data_mountpoint and first-stage rootfs early because they are used in init_msgs()
        self.data_mountpoint = rospy.get_param(
            "~data_mountpoint", self.data_mountpoint)
        self.first_stage_rootfs_device = rospy.get_param(
            "~first_stage_rootfs_device", self.first_stage_rootfs_device)

        self.init_msgs()

        self.valid_device_id_re = re.compile(r"^[a-zA-Z][\w]*$")

        self.run()


if __name__ == '__main__':
    SysMgr = SystemMgrNode()
