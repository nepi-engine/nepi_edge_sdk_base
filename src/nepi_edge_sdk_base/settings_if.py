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

    def publishSettingsCb(self, msg):
        self.publishSettingsStatus()

    def publishSettingsStatus(self):
        '''
        if self.getSettingsFunction is not None and not rospy.is_shutdown():
            current_settings = nepi_ros.sort_settings_alphabetically(self.getSettingsFunction()) 
        else:
            current_settings = nepi_ros.NONE_SETTINGS
        
        '''
        current_settings = rospy.get_param('~settings', self.init_settings )
        settings_msg = nepi_ros.create_msg_data_from_settings(current_settings)
        self.settings_status_pub.publish(settings_msg)


    def resetInitSettingsCb(self, msg):
        rospy.loginfo("SETTINGS_IF: Reseting Setting to Init Values")
        self.resetParamServer(do_updates = True)
        self.publishSettingsStatus()


    def updateSettingCb(self,msg):
        rospy.loginfo("Received settings update msg ")
        #rospy.loginfo(msg)
        setting = [msg.type_str,msg.name_str,msg.value_str]
        self.updateSetting(setting, update_status = True, update_param = True)


    def resetFactorySettings(self, update_params = True):
        rospy.loginfo("SETTINGS_IF: Applying Factory Settings")
        #rospy.loginfo(self.init_settings)
        for setting in self.factory_settings:
            self.updateSetting(setting,update_status = False, update_param = update_params)
        self.publishSettingsStatus()


    def updateSetting(self,new_setting,update_status = True, update_param = True):
        success = False
        #current_settings = rospy.get_param('~settings', self.init_settings)
        current_settings = nepi_ros.sort_settings_alphabetically(self.getSettingsFunction())
        if self.settingUpdateFunction != None:
            [name_match,type_match,value_match] = nepi_ros.compare_setting_in_settings(new_setting,current_settings)
            if value_match == False: # name_match would be true for value_match to be true
                rospy.loginfo("Will try to update setting " + str(new_setting))
                [success,msg] = nepi_ros.try_to_update_setting(new_setting,current_settings,self.cap_settings,self.settingUpdateFunction)
                rospy.loginfo(msg)
                if success:
                    if update_param:
                        updated_settings = nepi_ros.update_setting_in_settings(new_setting,current_settings)
                        rospy.set_param('~settings', updated_settings)
                    if update_status:
                        self.publishSettingsStatus() 
        else:
            rospy.loginfo("Settings updates ignored. No settings update function defined ")
        return success

    def initializeParamServer(self, do_updates = True):
        self.init_settings = rospy.get_param('~settings', self.factory_settings)
        rospy.set_param('~settings', self.init_settings)
        if do_updates:
            for setting in self.init_settings:
                self.updateSetting(setting, update_status = False, update_param = False)
            self.publishSettingsStatus()

    def resetParamServer(self, do_updates = True):
        rospy.set_param('~settings', self.init_settings)
        if do_updates:
            updateFromParamServer(self, update_status = False, update_param = False)
            self.publishSettingsStatus()

    def updateFromParamServer(self):
        rospy.loginfo("SETTINGS_IF: Updating Settings From Param Server")
        settings = rospy.get_param('~settings', self.init_settings )
        current_settings = self.getSettingsFunction()
        for setting in settings:
            self.updateSetting(setting,update_status = False, update_param = True)
        self.publishSettingsStatus()

    def __init__(self, capSettings=None, factorySettings=None,settingUpdateFunction=None, getSettingsFunction=None ):
           # Initialize Sensor Settings from Node

        self.settings_status_pub = rospy.Publisher('~settings_status', String, queue_size=1, latch=True)
        time.sleep(1)

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

            self.initializeParamServer(do_updates = False)     

            self.resetFactorySettings(update_params = False)
  
       # Update settings  and publish current values
        self.updateFromParamServer()

        rospy.Subscriber('~update_setting', SettingUpdate, self.updateSettingCb, queue_size=1) # start local callbac
        rospy.Subscriber('~publish_settings', Empty, self.publishSettingsCb, queue_size=1) # start local callback
        rospy.Subscriber('~reset_settings', Empty, self.resetInitSettingsCb, queue_size=1) # start local callback

        # Subscribe to global resets as well
        rospy.Subscriber('reset_settings', Empty, self.resetInitSettingsCb, queue_size=1) # start local callback


        # Start capabilities services
        rospy.Service('~settings_capabilities_query', SettingsCapabilitiesQuery, self.provide_capabilities)

 

