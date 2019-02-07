#!/usr/bin/env python
#TODO License

import rospy
import json
import os
import datetime

from num_sdk_msgs.msg import OotStatus
from num_sdk_base.srv import NepiDataDir, NepiDataDirResponse

NEPI_ROOT = "/home/nepi-usr"

class NepiCoreMgr:
    """The NEPI Core Manager Node of the Numurus core SDK.

    The role of this node is to manage all interaction between the
    NEPI-Bot and the rest of the system. This means both relaying
    commands and data from the cloud to the appropriate subsystems as
    well as relaying data from the device to the NEPI-Bot for upload.
    """

    NODE_NAME = "nepi_core_mgr"

    @staticmethod
    def _timestamp():
        return datetime.datetime.now().strftime("%Y_%m_%d_%H%M%S.%f")[:-3]

    @staticmethod
    def _verify_nepi_fs():
        if not (os.path.exists(NEPI_ROOT) and os.path.isdir(NEPI_ROOT)):
            rospy.logfatal("NEPI_ROOT ({}) does not exist or is not a directory!".format(NEPI_ROOT))
            return False

        # TODO: add more once ICD is nailed down.
        subdirs = [
                os.path.join(NEPI_ROOT, "data"),
        ]

        for subdir in subdirs:
            if not (os.path.exists(subdir) and os.path.isdir(subdir)):
                rospy.logfatal("NEPI directory {} does not exist or is not a directory!".format(subdir))
                return False

        return True

    def handle_oot_status(self, msg):
        rospy.loginfo("Received oot_status message, fix time (s): {}".format(msg.navsat_fix_time.secs))

        oot_status_dict = {
                "timestamp": msg.timestamp.secs + msg.timestamp.nsecs/1e9,
                "serial_num": msg.serial_num,
                "sw_rev": msg.sw_rev,
                "navsat_fix_time": msg.navsat_fix_time.secs + msg.navsat_fix_time.nsecs/1e9,
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "heading": msg.heading,
                "batt_charge": msg.batt_charge,
                "bus_voltage": msg.bus_voltage,
                "temperature": msg.temperature,
                "trig_wake_count": msg.trig_wake_count,
                "wake_event_type": msg.wake_event_type,
                "wake_event_id": msg.wake_event_id,
                "task_index": msg.task_index,
                "trig_cfg_index": msg.trig_cfg_index,
                "rule_cfg_index": msg.rule_cfg_index,
                "sensor_cfg_index": msg.sensor_cfg_index,
                "node_cfg_index": msg.node_cfg_index,
                "state_flags": msg.state_flags,
        }

        if not self._verify_nepi_fs():
            rospy.logfatal("Failed to write sys_stat.json due to NEPI FS error (see previous messages).")
            return

        data_dir = os.path.join(NEPI_ROOT, "data", self._timestamp())
        if os.path.exists(data_dir):
            rospy.logwarn("Data directory ({}) already exists, copying files anyway...".format(data_dir))
        else:
            os.mkdir(data_dir)
        self.current_data_dir = data_dir

        with open(os.path.join(data_dir, "sys_stat.json"), "w") as f:
            json.dump(oot_status_dict, f, indent=4)

    def get_current_data_dir(self, req):
        return NepiDataDirResponse(self.current_data_dir)

    def run(self):
        rospy.spin()

    def __init__(self):
        rospy.init_node(self.NODE_NAME)

        rospy.loginfo("Starting the {} node".format(self.NODE_NAME))

        rospy.Subscriber("oot_status", OotStatus, self.handle_oot_status)

        self.current_data_dir = ""
        rospy.Service("get_current_data_dir", NepiDataDir, self.get_current_data_dir)

        self.run()

if __name__ == "__main__":
    node = NepiCoreMgr()
