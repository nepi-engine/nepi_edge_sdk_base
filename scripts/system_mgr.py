#!/usr/bin/env python
import os
import shutil
import random # Only necessary for simulation of temperature data
from collections import deque
import re

import subprocess

import rospy

from std_msgs.msg import String, Empty, Float32
from num_sdk_msgs.msg import SystemStatus, SystemDefs, WarningFlags, StampedString, SaveData
from num_sdk_msgs.srv import SystemDefsQuery, SystemDefsQueryResponse, OpEnvironmentQuery, OpEnvironmentQueryResponse

from num_sdk_base.save_cfg_if import SaveCfgIF

BYTES_PER_MEGABYTE = 2**20

class SystemMgrNode():
	NODE_NAME = "system_mgr"

	STATUS_PERIOD = 1.0 # TODO: Configurable update period?

	DISK_FULL_MARGIN_MB = 250 # MB TODO: Configurable?

	SYS_ENV_PATH = "/opt/numurus/sys_env.bash"
	FW_VERSION_PATH = "/opt/numurus/ros/etc/fw_version.txt"

	status_msg = SystemStatus()
	system_defs_msg = SystemDefs()

	data_mountpoint = "/home/numurus_user/data"

	#disk_usage_deque = deque(maxlen=10)
	disk_usage_deque = deque(maxlen=3) # Shorter period for more responsive updates

	def add_info_string(self, string, level):
		self.status_msg.info_strings.append(StampedString(timestamp=rospy.get_rostime(), payload=string, priority=level))

	def get_device_sn(self):
		with open(self.SYS_ENV_PATH, "r") as f:
			for line in f:
				if line.startswith("export DEVICE_SN="):
					return line.split('=')[1].rstrip()
		return "undefined"

	def get_fw_rev(self):
		if (os.path.exists(self.FW_VERSION_PATH) and (os.path.getsize(self.FW_VERSION_PATH) > 0)):
			with open(self.FW_VERSION_PATH, "r") as f:
				return f.readline()
		return "UNSPECIFIED"

	def update_temperatures(self):
		# Get the current temperatures
		
		# TODO: Should the temperature sensor or the entire subproc. cmd line be configurable? 
		temp_string_mdegC = subprocess.check_output(["cat", "/sys/class/thermal/thermal_zone0/temp"])
		self.status_msg.temperatures[0] = float(temp_string_mdegC) / 1000.0

		# Check for temperature warnings and do thermal throttling
		throttle_ratio_min = 1.0
		for i,t in enumerate(self.status_msg.temperatures):
			if (t > self.system_defs_msg.critical_temps[i]):
				# Critical implies high
				self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = True
				self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
				# Make sure to send a user string
				self.add_info_string(WarningFlags.CRITICAL_TEMPERATURE_STRING, StampedString.PRI_HIGH)
				# Set the throttle ratio to 0% globally
				rospy.logerr_throttle(10, "%s: temperature = %f", WarningFlags.CRITICAL_TEMPERATURE_STRING, t)
				throttle_ratio_min = 0.0
			else:
				self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = False
				if (t > self.system_defs_msg.warning_temps[i]):
					self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
					throttle_ratio_i = 1.0 - ((t - self.system_defs_msg.warning_temps[i]) / (self.system_defs_msg.critical_temps[i] - self.system_defs_msg.warning_temps[i]))
					rospy.logwarn_throttle(10, "%s: temperature = %f", WarningFlags.HIGH_TEMPERATURE_STRING, t)
					throttle_ratio_min = min(throttle_ratio_i, throttle_ratio_min)
				else:
					self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = False
			# If a new thermal throttle ratio was computed, publish it globally
			if (throttle_ratio_min != self.current_throttle_ratio):
				self.throttle_ratio_pub.publish(Float32(throttle_ratio_min))
				self.current_throttle_ratio = throttle_ratio_min
				rospy.logwarn("New thermal rate throttle value: %f%%", self.current_throttle_ratio)

	def update_storage(self):
		statvfs = os.statvfs(self.data_mountpoint)
		disk_free = float(statvfs.f_frsize) * statvfs.f_bavail / BYTES_PER_MEGABYTE # In MB
		self.status_msg.disk_usage = self.system_defs_msg.disk_capacity - disk_free

		self.disk_usage_deque.append(self.status_msg.disk_usage)
		self.status_msg.storage_rate = (self.disk_usage_deque[-1] - self.disk_usage_deque[0]) / (len(self.disk_usage_deque)*self.STATUS_PERIOD)

		# Check for disk warnings
		if (disk_free < self.DISK_FULL_MARGIN_MB):
			self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = True
			self.add_info_string("Max disk usage exceeded", StampedString.PRI_HIGH)
			# Force all nodes to stop data saving
			self.save_data_pub.publish(save_continuous=False, save_raw=False)
		else:
			self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = False


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
			rospy.logwarn("Refusing to clear data folder because data saving is currently enabled")
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
		        rospy.logwarn('Failed to delete %s. Reason: %s' % (file_path, e))

	def set_op_environment(self, msg):
		if (msg.data != OpEnvironmentQueryResponse.OP_ENV_AIR) and (msg.data != OpEnvironmentQueryResponse.OP_ENV_WATER):
			rospy.logwarn("Setting environment parameter to a non-standard value: %s", msg.data)
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
		self.add_info_string("Device ID updated - Requires device reboot", StampedString.PRI_ELEVATED)

	def handle_system_error_msg(self, msg):
		self.add_info_string(msg.data, StampedString.PRI_HIGH)

	def provide_system_defs(self, req):
		return SystemDefsQueryResponse(self.system_defs_msg)

	def provide_op_environment(self, req):
		# Just proxy the param server
		return OpEnvironmentQueryResponse(rospy.get_param("~op_environment", OpEnvironmentQueryResponse.OP_ENV_AIR))

	def init_msgs(self):
		self.system_defs_msg.firmware_version = self.get_fw_rev()

		self.system_defs_msg.device_sn = self.get_device_sn()

		#TODO: Determine how many temperature readings we have. On Jetson, for example
		#      there are 8 "thermal zones" in /sys/class/thermal/
		self.system_defs_msg.temperature_sensor_names.append('CPU Zone 0')
		#TODO: Configurable warning/error temperatures
		self.system_defs_msg.warning_temps.append(50.0)
		self.system_defs_msg.critical_temps.append(70.0)

		statvfs = os.statvfs(self.data_mountpoint)
		self.system_defs_msg.disk_capacity = statvfs.f_frsize * statvfs.f_blocks / BYTES_PER_MEGABYTE     # Size of data filesystem in Megabytes

		for i in self.system_defs_msg.temperature_sensor_names:
			self.status_msg.temperatures.append(0.0)

		# TODO: Should this be queried somehow e.g., from the param server
		self.status_msg.save_all_enabled = False

	def updateFromParamServer(self):
		op_env = rospy.get_param("~op_environment", OpEnvironmentQueryResponse.OP_ENV_AIR)
		# Publish it to all subscribers (which includes this node) to ensure the parameter is applied
		self.set_op_env_pub.publish(String(op_env))

		self.data_mountpoint = rospy.get_param("~data_mountpoint", self.data_mountpoint)

	def run(self):
		# Want to update the op_environment (from param server) through the whole system once at
		# start-up, but the only reasonable way to do that is to delay long enough to let all nodes start
		rospy.sleep(3)
		self.updateFromParamServer()

		rospy.Timer(rospy.Duration(self.STATUS_PERIOD), self.publish_periodic_status)
		rospy.spin()

	def __init__(self):
		rospy.loginfo("Starting " + self.NODE_NAME + "node")
		rospy.init_node(self.NODE_NAME)

		status_period = rospy.Duration(1) # TODO: Configurable rate?

		# Announce published topics
		self.status_pub = rospy.Publisher('system_status', SystemStatus, queue_size=1)
		self.store_params_pub = rospy.Publisher('store_params', String, queue_size = 10)
		self.throttle_ratio_pub = rospy.Publisher('apply_throttle', Float32, queue_size=3)
		self.set_op_env_pub = rospy.Publisher('set_op_environment', String, queue_size=3)
		self.save_data_pub = rospy.Publisher('save_data', SaveData, queue_size=1) # For auto-stop of save data; disk full protection

		self.current_throttle_ratio = 1.0

		# Subscribe to topics
		rospy.Subscriber('save_data', SaveData, self.set_save_status)
		rospy.Subscriber('clear_data_folder', Empty, self.clear_data_folder)
		rospy.Subscriber('set_op_environment', String, self.set_op_environment)

		rospy.Subscriber('set_device_id', String, self.set_device_id) # Public ns

		rospy.Subscriber('submit_system_error_msg', String, self.handle_system_error_msg)

		# Advertise services
		rospy.Service('system_defs_query', SystemDefsQuery, self.provide_system_defs)
		rospy.Service('op_environment_query', OpEnvironmentQuery, self.provide_op_environment)

		self.save_cfg_if = SaveCfgIF(updateParamsCallback=None, paramsModifiedCallback=self.updateFromParamServer)

                # Need to get the data_mountpoint early because it is used in init_msgs()
		self.data_mountpoint = rospy.get_param("~data_mountpoint", self.data_mountpoint)
		self.init_msgs()

		self.valid_device_id_re = re.compile(r"^[a-zA-Z][\w]*$")

		self.run()

if __name__ == '__main__':
	SysMgr = SystemMgrNode()
