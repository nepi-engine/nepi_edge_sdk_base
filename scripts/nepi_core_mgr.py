#!/usr/bin/env python
#TODO License

import rospy

from num_sdk_msgs.msg import OotStatus

class NepiCoreMgr:
    """The NEPI Core Manager Node of the Numurus core SDK.

    The role of this node is to manage all interaction between the
    NEPI-Bot and the rest of the system. This means both relaying
    commands and data from the cloud to the appropriate subsystems as
    well as relaying data from the device to the NEPI-Bot for upload.
    """

    NODE_NAME = "nepi_core_mgr"

    def handle_oot_status(self, msg):
        # TODO: write to NEPI FS
        rospy.loginfo("Received oot_status message, fix time (s): {}".format(msg.navsat_fix_time.secs))

    def run(self):
        rospy.spin()

    def __init__(self):
        rospy.init_node(self.NODE_NAME)

        rospy.loginfo("Starting the {} node".format(self.NODE_NAME))

        rospy.Subscriber("oot_status", OotStatus, self.handle_oot_status)

        self.run()

if __name__ == "__main__":
    node = NepiCoreMgr()
