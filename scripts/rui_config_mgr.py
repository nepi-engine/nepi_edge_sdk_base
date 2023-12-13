#!/usr/bin/env python
#
# NEPI Dual-Use License
# Project: nepi_edge_sdk_base
#
# This license applies to any user of NEPI Engine software
#
# Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
# see https://github.com/numurus-nepi/nepi_edge_sdk_base
#
# This software is dual-licensed under the terms of either a NEPI software developer license
# or a NEPI software commercial license.
#
# The terms of both the NEPI software developer and commercial licenses
# can be found at: www.numurus.com/licensing-nepi-engine
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - https://www.numurus.com/licensing-nepi-engine
# - mailto:nepi@numurus.com
#
#
import rospy

from std_msgs.msg import UInt8
from nepi_ros_interfaces.msg import RUISettings

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

class RUICfgMgrNode:
    NODE_NAME = "rui_config_mgr"
    DEFAULT_IMAGE_QUALITY = 95

    def publish_settings(self):
        # Gather all settings for the message
        self.settings_msg.streaming_image_quality = rospy.get_param("~streaming_image_quality", self.DEFAULT_IMAGE_QUALITY)
        self.settings_msg.nepi_hb_auto_offload_visible = rospy.get_param("~nepi_hb_auto_offload_visible", False)

        # Publish it
        self.settings_pub.publish(self.settings_msg)

    def set_streaming_image_quality_cb(self, msg):
        if (msg.data < 1 or msg.data > 100):
            rospy.logerr("Invalid image qualtiy: %u... ignoring", msg.data)
            return

        rospy.loginfo("Setting streaming image quality to %u", msg.data)
        rospy.set_param("~streaming_image_quality", msg.data)
        self.publish_settings() # Make sure to always publish settings updates

    def __init__(self):
        rospy.init_node(self.NODE_NAME)
        rospy.loginfo("Starting " + self.NODE_NAME + " node")

        self.settings_pub = rospy.Publisher('~settings', RUISettings, queue_size=1, latch=True)
        self.settings_msg = RUISettings()
        self.publish_settings() # Do it once so that latch works on next connection

        rospy.Subscriber('~set_streaming_image_quality', UInt8, self.set_streaming_image_quality_cb)

        self.save_cfg_if = SaveCfgIF(updateParamsCallback=None, paramsModifiedCallback=None)

        rospy.spin()

if __name__ == '__main__':
  RUICfgMgr = RUICfgMgrNode()
