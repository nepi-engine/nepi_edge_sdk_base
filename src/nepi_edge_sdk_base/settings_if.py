#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy
import time

from nepi_edge_sdk_base import nepi_ros
from std_msgs.msg import String, Empty
from nepi_ros_interfaces.msg import SettingUpdate
from nepi_ros_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryResponse

'''
Basic interface for the global and private settings topics.
'''
class SettingsIF(object):

    caps_settings = None
    factorySettings = None
    initSettings = None
    settingUpdateFunction = None

    capabilities_report = SettingsCapabilitiesQueryResponse()

    def provide_capabilities(self, _):
        return self.capabilities_report

    def resetInitSettingsCb(self, msg):
        #rospy.loginfo(msg)
        rospy.loginfo("Received settings reset msg")
        self.resetInitSettings()

    def resetInitSettings(self):
        #rospy.loginfo(self.init_settings)
        for setting in self.init_settings:
            self.updateSetting(setting)


    def resetFactorySettings(self):
        #rospy.loginfo(self.init_settings)
        for setting in self.factory_settings:
            self.updateSetting(setting)


    def updateSettingCb(self,msg):
        rospy.loginfo("Received settings update msg ")
        #rospy.loginfo(msg)
        new_setting = [msg.type_str,msg.name_str,msg.value_str]
        self.updateSetting(new_setting)


    def updateSetting(self,new_setting,update_status = True):
        success = False
        current_settings = nepi_ros.sort_settings_alphabetically(self.getSettingsFunction())
        if self.settingUpdateFunction != None:
            [name_match,type_match,value_match] = nepi_ros.compare_setting_in_settings(new_setting,current_settings)
            if not value_match: # name_match would be true for value_match to be true
                rospy.loginfo("Will try to update setting " + str(new_setting))
                [success,msg] = nepi_ros.try_to_update_setting(new_setting,current_settings,self.cap_settings,self.settingUpdateFunction)
                rospy.loginfo(msg)
                self.publishSettingsStatus() 
            else:
                rospy.loginfo("Skipping setting update " + str(new_setting) + " because it matches current setting")
        else:
            rospy.loginfo("Settings updates ignored. No settings update function defined ")

        return success


    def publishSettingsStatus(self):
        if self.getSettingsFunction is not None:
            current_settings = nepi_ros.sort_settings_alphabetically(self.getSettingsFunction()) 
            rospy.set_param('~settings', current_settings)
        else:
            current_settings = nepi_ros.NONE_SETTINGS
        settings_msg = nepi_ros.create_msg_data_from_settings(current_settings)
        self.settings_status_pub.publish(settings_msg)

   
    def updateFromParamServer(self):
        settings = rospy.get_param('~settings', self.init_settings )
        current_settings = self.getSettingsFunction()
        for setting in settings:
            if self.settingUpdateFunction != None:
                [name_match,type_match,value_match] = nepi_ros.compare_setting_in_settings(setting,current_settings)
                if not value_match: # name_match would be true for value_match to be true
                    self.updateSetting(setting)

    def __init__(self, capSettings=None, factorySettings=None,settingUpdateFunction=None, getSettingsFunction=None ):
           # Initialize Sensor Settings from Node
        if capSettings is None:
            self.cap_settings = nepi_ros.NONE_SETTINGS
            self.capabilities_report.settings = False
            self.factory_settings = nepi_ros.NONE_SETTINGS
            self.init_settings = rospy.get_param('~settings', nepi_ros.NONE_SETTINGS)
            rospy.set_param('~settings', self.init_settings )
        else:
            self.cap_settings = nepi_ros.sort_settings_alphabetically(capSettings)    
            capSettings_msg = nepi_ros.create_msg_data_from_settings(self.cap_settings)
            self.capabilities_report.settings_options = capSettings_msg

            if factorySettings is None:
                self.factory_settings = nepi_ros.NONE_SETTINGS
            else:
                self.factory_settings = nepi_ros.sort_settings_alphabetically(factorySettings) 

            if settingUpdateFunction is None:
                self.settingUpdateFunction = nepi_ros.UPDATE_NONE_SETTINGS_FUNCTION
            else:
                self.settingUpdateFunction = settingUpdateFunction
            
            if getSettingsFunction is None:
                self.getSettingsFunction = nepi_ros.GET_NONE_SETTINGS_FUNCTION
            else:
                self.getSettingsFunction = getSettingsFunction
            # Set init values for resets. Updated saveConfigCb() on save_config msg
            self.init_settings = rospy.get_param('~settings', self.factory_settings)
            rospy.set_param('~settings', self.init_settings )


        rospy.Subscriber('~update_setting', SettingUpdate, self.updateSettingCb, queue_size=1) # start local callbac
        rospy.Subscriber('~reset_settings', Empty, self.resetInitSettingsCb, queue_size=1) # start local callback

        # Subscribe to global resets as well
        rospy.Subscriber('reset_settings', Empty, self.resetInitSettingsCb, queue_size=1) # start local callback


        self.settings_status_pub = rospy.Publisher('~settings_status', String, queue_size=1, latch=True)
        time.sleep(1)
        self.publishSettingsStatus()

        # Start capabilities services
        rospy.Service('~settings_capabilities_query', SettingsCapabilitiesQuery, self.provide_capabilities)
