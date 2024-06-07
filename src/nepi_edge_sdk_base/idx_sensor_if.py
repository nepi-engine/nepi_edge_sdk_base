#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import time
import threading
import subprocess
import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
import open3d as o3d

from std_msgs.msg import UInt8, Float32, Bool, Empty, String
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow, SaveData, SaveDataRate, SaveDataStatus
from nepi_ros_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryResponse, NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_nex
from nepi_edge_sdk_base import nepi_img
from nepi_edge_sdk_base import nepi_pc

class ROSIDXSensorIF:
    # Default Global Values
    RESOLUTION_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    FRAMERATE_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    SETTINGS_STATE_LIST = []
    UPDATE_NAVPOSE_RATE_HZ = 10
    CHECK_DATA_SAVE_RATE_HZ = 40
    
    # Backup Factory Control Values 
    FACTORY_CONTROLS = dict( controls_enable = True,
        auto_adjust = False,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.5,
        resolution_mode = 1, # LOW, MED, HIGH, MAX
        framerate_mode = 1, # LOW, MED, HIGH, MAX
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        min_range_m = 0.0,
        max_range_m = 1.0,
        frame_3d = 'nepi_center_frame'
    )

    FACTORY_SAVE_DATA_CONFIGS = dict( color_2d_image = "1",
        bw_2d_image = "0",
        depth_map = "0",
        depth_image = "0",
        pointcloud_image = "0",
        pointcloud = "0",
    )

    # Define class variables
    factory_controls = None
    init_controls_enable = None 
    init_auto_adjust = None
    init_brightness_ratio = None
    init_contrast_ratio = None
    init_threshold_ratio = None
    init_resolution_mode = None
    init_framerate_mode = None
    init_min_range = None
    init_max_range = None
    init_frame_3d = None


    init_zoom_ratio = 0.5
    init_rotate_ratio = 0.5
    init_tilt_ratio = 0.5

    zoom_ratio = init_zoom_ratio
    rotate_ratio = init_rotate_ratio
    tilt_ratio = init_rotate_ratio
    render_controls = [zoom_ratio,rotate_ratio,tilt_ratio]


    caps_settings = None
    factory_settings = None
    init_settings = None
    update_settings_function = None

    data_products = []

    factory_save_data_configs = []
    init_save_data_configs = []
    factory_save_data_enabled = False
    init_save_data_enabled = False
    factory_save_data_nav_enabled = False
    init_save_data_nav_enabled = False
    save_data_nav_enabled = False
    save_data_prefix = ""
    init_save_data_prefix = ""

    save_data_if = None
    save_cfg_if = None

    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ
    check_data_save_interval_sec = float(1)/CHECK_DATA_SAVE_RATE_HZ

    def resetFactoryCb(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Factory Resetting IDX Sensor Controls")
        self.resetFactory()

    def resetFactory(self):
        rospy.set_param('~idx/auto', self.factory_controls.get('auto_adjust'))
        self.status_msg.auto_adjust = self.factory_controls.get('auto_adjust')
        
        rospy.set_param('~idx/brightness', self.factory_controls.get('brightness_ratio'))
        self.status_msg.brightness = self.factory_controls.get('brightness_ratio')
        
        rospy.set_param('~idx/contrast', self.factory_controls.get('contrast_ratio'))
        self.status_msg.contrast = self.factory_controls.get('contrast_ratio')
        
        rospy.set_param('~idx/thresholding', self.factory_controls.get('threshold_ratio'))
        self.status_msg.thresholding = self.factory_controls.get('threshold_ratio')
        
        rospy.set_param('~idx/resolution_mode', self.factory_controls.get('resolution_mode'))
        self.status_msg.resolution_mode = self.factory_controls.get('resolution_mode')
        
        rospy.set_param('~idx/framerate_mode', self.factory_controls.get('framerate_mode'))
        self.status_msg.framerate_mode = self.factory_controls.get('framerate_mode')

        rospy.set_param('~idx/range_window/start_range_ratio', self.factory_controls.get('start_range_ratio'))
        rospy.set_param('~idx/range_window/stop_range_ratio', self.factory_controls.get('stop_range_ratio'))
        self.status_msg.range_window.start_range = self.factory_controls.get('start_range_ratio')
        self.status_msg.range_window.stop_range =  self.factory_controls.get('stop_range_ratio')
        
        rospy.set_param('~idx/frame_3d', self.factory_controls.get('frame_3d'))
        self.status_msg.frame_3d = self.factory_controls.get('frame_3d')

        self.updateSettings(self.factory_settings)

        self.updateSaveDataConfigs(self.factory_save_data_configs)
        self.setSaveDataEnable(self.factory_save_data_enabled)
        self.setSaveDataNavEnable(self.factory_save_data_nav_enabled)
        self.setSaveDataPrefix(self.init_save_data_prefix)

        self.zoom_ratio = self.init_zoom_ratio
        self.rotate_ratio = self.init_rotate_ratio
        self.tilt_ratio = self.init_rotate_ratio
        self.render_controls = [self.zoom_ratio,self.rotate_ratio,self.tilt_ratio]
        self.status_msg.zoom = self.zoom_ratio
        self.status_msg.rotate = self.rotate_ratio
        self.status_msg.tilt = self.tilt_ratio
        self.updateAndPublishStatus(do_updates=False)

        self.updateFromParamServer()


    def saveConfigCb(self, msg):  # Just update class init values. Saving done by Config IF system
        self.init_auto_adjust = rospy.get_param('~idx/auto', self.factory_controls.get('auto_adjust'))
        self.init_brightness_ratio = rospy.get_param('~idx/brightness', self.factory_controls.get('brightness_ratio'))
        self.init_contrast_ratio = rospy.get_param('~idx/contrast', self.factory_controls.get('contrast_ratio'))
        self.init_threshold_ratio = rospy.get_param('~idx/thresholding', self.factory_controls.get('threshold_ratio'))
        self.init_resolution_mode = rospy.get_param('~idx/resolution_mode', self.factory_controls.get('resolution_mode'))
        self.init_framerate_mode = rospy.get_param('~idx/framerate_mode', self.factory_controls.get('framerate_mode'))
        self.init_start_range_ratio = rospy.get_param('~idx/range_window/start_range_ratio', self.factory_controls.get('start_range_ratio'))
        self.init_stop_range_ratio = rospy.get_param('~idx/range_window/stop_range_ratio', self.factory_controls.get('stop_range_ratio'))
        self.init_frame_3d = rospy.get_param('~idx/frame_3d', self.factory_controls.get('frame_3d'))
        self.init_settings = rospy.get_param('~idx/settings',self.factory_settings)
        self.init_save_data_configs = rospy.get_param('~idx/save_data_configs',self.factory_save_data_configs)
        self.init_save_data_enabled = rospy.get_param('~idx/save_data_enabled',self.factory_save_data_enabled)
        self.init_save_data_nav_enabled = rospy.get_param('~idx/save_data_enabled',self.factory_save_data_nav_enabled)
        self.updateFromParamServer()



    def resetControlsCb(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Resetting IDX Sensor Controls")
        self.resetControls()

    def resetControls(self):
        rospy.set_param('~idx/auto', self.init_auto_adjust)
        self.status_msg.auto_adjust = self.init_auto_adjust
        
        rospy.set_param('~idx/brightness', self.init_brightness_ratio)
        self.status_msg.brightness = self.init_brightness_ratio
        
        rospy.set_param('~idx/contrast', self.init_contrast_ratio)
        self.status_msg.contrast = self.init_contrast_ratio
        
        rospy.set_param('~idx/thresholding', self.init_threshold_ratio)
        self.status_msg.thresholding = self.init_threshold_ratio
        
        rospy.set_param('~idx/resolution_mode', self.init_resolution_mode)
        self.status_msg.resolution_mode = self.init_resolution_mode
        
        rospy.set_param('~idx/framerate_mode', self.init_framerate_mode)
        self.status_msg.framerate_mode = self.init_framerate_mode

        rospy.set_param('~idx/range_window/start_range_ratio', self.init_start_range_ratio)
        rospy.set_param('~idx/range_window/stop_range_ratio', self.init_stop_range_ratio)
        self.status_msg.min_range_m = self.init_min_range_m
        self.status_msg.max_range_m = self.init_max_range_m
        
        rospy.set_param('~idx/frame_3d', self.init_frame_3d)
        self.status_msg.frame_3d = self.init_frame_3d

        self.zoom_ratio = self.init_zoom_ratio
        self.rotate_ratio = self.init_rotate_ratio
        self.tilt_ratio = self.init_rotate_ratio
        self.render_controls = [self.zoom_ratio,self.rotate_ratio,self.tilt_ratio]
        self.status_msg.zoom = self.zoom_ratio
        self.status_msg.rotate = self.rotate_ratio
        self.status_msg.tilt = self.tilt_ratio
        self.updateAndPublishStatus(do_updates=False)


        self.updateFromParamServer()

    def resetSettingsCb(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Resetting IDX Sensor Settings")
        self.resetSettings()

    def resetSettings(self):
        rospy.loginfo(self.init_settings)
        self.updateSettings(self.init_settings)
        self.updateFromParamServer()


    def updateSettingsCb(self,msg):
        #rospy.loginfo("Received settings update msg ")
        #rospy.loginfo(msg)
        new_settings = nepi_nex.parse_settings_msg_data(msg.data)
        rospy.loginfo("Received settings update list")
        #rospy.loginfo(new_settings)
        self.updateSettings(new_settings)


    def updateSettings(self,new_settings):
        success = False
        if self.getCurrentSettings() is not None:
            current_settings = nepi_nex.sort_settings_alphabetically(self.getCurrentSettings(),1)
            if self.update_settings_function != None:
                for setting in new_settings:
                    [name_match,type_match,value_match] = nepi_nex.compare_setting_in_settings(setting,current_settings)
                    if not value_match: # name_match would be true for value_match to be true
                        rospy.loginfo("Will try to update setting " + str(setting))
                        [success,msg] = nepi_nex.try_to_update_setting(setting,current_settings,self.cap_settings,self.update_settings_function)
                        rospy.loginfo(msg)
                    else:
                        rospy.loginfo("Skipping setting update " + str(setting) + " because it matches current setting")
            else:
                rospy.loginfo("Settings updates ignored. No settings update function defined ")
        else:
            rospy.loginfo("Skipping settings update request. getCurrentSettings function not defined")
        if self.getSettings is not None:
            current_settings = nepi_nex.sort_settings_alphabetically(self.getSettings(),1) 
            rospy.set_param('~idx/settings', current_settings)
        else:
            current_settings = nepi_nex.NONE_SETTINGS
        settings_msg = nepi_nex.create_msg_data_from_settings(current_settings)
        self.status_msg.settings = settings_msg 
        self.updateAndPublishStatus(do_updates=False) # Updated inline here
        return success


    def resetSaveDataCb(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Resetting IDX Save Data Config")
        self.resetSaveData()

    def resetSaveData(self):
        rospy.loginfo(self.init_save_data_configs)
        self.updateSaveDataConfigs(self.init_save_data_configs)
        self.setSaveDataPrefix(self.init_save_data_prefix)
        self.setSaveDataNavEnable(self.init_save_data_nav_enabled)
        self.setSaveDataEnable(self.init_save_data_enabled)
        self.updateFromParamServer()


    def updateSaveDataConfigsCb(self,msg):
        #rospy.loginfo("Received save data configs update msg ")
        #rospy.loginfo(msg)
        new_save_configs = nepi_nex.parse_save_configs_msg(msg.data)
        rospy.loginfo("Received save data configs update list")
        rospy.loginfo(new_save_configs)
        self.updateSaveDataConfigs(new_save_configs)


    def updateSaveDataConfigs(self,new_save_configs):
        success = False
        if self.save_data_if is not None:
            configs = self.save_data_if.getSaveDataConfigs()
            if configs is not None:
                for config in new_save_configs:
                    data_product = config[0]
                    rate = float(config[1])
                    if data_product in self.save_data_if.data_rate_dict.keys():
                        if self.save_data_if.data_rate_dict[data_product][0] != rate:
                            if rate >= 0 and rate <= 100:
                                self.save_data_if.update_save_data_rate(data_product,rate)
                                configs = nepi_nex.update_config_in_save_configs(config,configs)
                                success = True
                                save_nav = rospy.get_param('~idx/save_data_nav_enabled', self.init_save_data_nav_enabled)
                                if save_nav:
                                    configs =  rospy.get_param('~idx/save_data_configs',  self.init_save_data_configs)
                                    max_rate = 0
                                    for config in configs:
                                        if float(config[1]) > max_rate:
                                            max_rate = float(config[1])
                                    nav_rate_msg = SaveDataRate()
                                    nav_rate_msg.data_product=self.nav_data_product
                                    nav_rate_msg.save_rate_hz = max_rate
                                    rospy.loginfo(nav_rate_msg)
                                    self.set_nav_save_rate_pub.publish(nav_rate_msg)
                            else:
                                rospy.loginfo("Ignoring save config update " + str(config) + " as it is out of bounds")
                                success = False
                        else:
                            rospy.loginfo("Ignoring save config update " + str(config) + " as it matches current value")
                            success = False
                    else:
                        rospy.loginfo("Ignoring save config update " + str(config) + " as it is not supported data product")
                        success = False
                rospy.set_param('~idx/save_data_configs',  configs)
                self.status_msg.save_data_configs = nepi_nex.create_save_configs_msg(configs)
                self.updateAndPublishStatus(do_updates=False) # Updated inline above
        return success   


    def setSaveDataPrefixCb(self,msg):
        #rospy.loginfo("Received save data prefix msg ")
        #rospy.loginfo(msg)
        new_save_prefix = msg.data
        rospy.loginfo("Received save data prefix update " + str(new_save_prefix))
        self.setSaveDataPrefix(new_save_prefix)

    def setSaveDataPrefix(self,new_save_prefix):
        success = False
        if self.save_data_if is not None:
            if new_save_prefix.find('\\') == -1:
                if new_save_prefix != self.save_data_if.save_data_prefix:
                    self.save_data_if.update_save_data_path_and_prefix(new_save_prefix)
                    self.save_data_prefix = new_save_prefix
                    self.status_msg.save_data_prefix = new_save_prefix
                    success = True
                    save_nav = rospy.get_param('~idx/save_data_nav_enabled', self.init_save_data_nav_enabled)
                    if save_nav:
                        self.set_nav_save_prefix_pub.publish(self.save_data_prefix)
            else:
                rospy.loginfo("Ignoring save prefix update as it contained a backslash")
        self.updateAndPublishStatus(do_updates=False) # Updated inline here
        return success

    def setSaveDataEnableCb(self,msg):
        #rospy.loginfo("Received save data enabled msg ")
        #rospy.loginfo(msg)
        new_save_enable = msg.data
        rospy.loginfo("Received save data enable update " + str(new_save_enable))
        self.setSaveDataEnable(new_save_enable)

    def setSaveDataEnable(self,new_save_enable):
        success = False
        if self.save_data_if is not None:
            if new_save_enable != self.save_data_if.save_continuous:
                self.save_data_if.save_data_enable(save_continuous = new_save_enable)
                rospy.set_param('~idx/save_data_enabled', new_save_enable)
                self.status_msg.save_data_enabled = new_save_enable
                success = True
                # Update nav saving if enabled.
                save_nav = rospy.get_param('~idx/save_data_nav_enabled', self.init_save_data_nav_enabled)
                rospy.loginfo("Save Nav : " + str(save_nav))
                if save_nav:
                    nav_save_msg = SaveData()
                    nav_save_msg.save_continuous = new_save_enable
                    nav_save_msg.save_raw = False
                    #rospy.loginfo(nav_save_msg)
                    self.set_nav_save_enable_pub.publish(nav_save_msg)
            else:
                rospy.loginfo("Ignoring save nav data enable update since it matches current setting")
        self.updateAndPublishStatus(do_updates=False) # Updated inline here
        return success

    def setSaveDataNavEnableCb(self,msg):
        #rospy.loginfo("Received save data nav enabled msg ")
        #rospy.loginfo(msg)
        new_save_enable = msg.data
        rospy.loginfo("Received save data nav enable update " + str(new_save_enable))
        self.setSaveDataNavEnable(new_save_enable)

    def setSaveDataNavEnable(self,new_save_enable):
        success = False
        if new_save_enable != rospy.get_param('~idx/save_data_nav_enabled', self.init_save_data_nav_enabled):
            rospy.set_param('~idx/save_data_nav_enabled', new_save_enable)
            self.status_msg.save_data_nav_enabled = new_save_enable
            success = True
        else:
            rospy.loginfo("Ignoring save data enable update since it matches current setting")
        self.updateAndPublishStatus(do_updates=False) # Updated inline here
        return success



    # Define local IDX Control callbacks
    def setControlsEnableCb(self, msg):
        new_controls_enable = msg.data
        rospy.loginfo("new_controls_enable")
        if self.setControlsEnable is not None:
            # Call the parent's method and update ROS param as necessary
            # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
            status, err_str = self.setControlsEnable(new_controls_enable)
            if status is True:
                rospy.set_param('~idx/controls_enable', new_controls_enable)
                self.status_msg.controls_enable = new_controls_enable
                self.updateAndPublishStatus(do_updates=False) # Updated inline here
                if new_controls_enable:
                    rospy.loginfo("Enabling IDX Controls")
                else:
                    rospy.loginfo("Disabling IDX Controls")
                    # Reset brightness, contrast, and threshold to factory settings
                    rospy.set_param('~idx/brightness', self.factory_controls["brightness_ratio"])
                    self.status_msg.brightness = self.factory_controls["brightness_ratio"]
                    
                    rospy.set_param('~idx/contrast', self.factory_controls["contrast_ratio"])
                    self.status_msg.contrast = self.factory_controls["contrast_ratio"]
                    
                    rospy.set_param('~idx/thresholding', self.factory_controls["threshold_ratio"])
                    self.status_msg.thresholding = self.factory_controls["threshold_ratio"]

                    self.updateFromParamServer()
        else:
            rospy.loginfo("Ignoring set controls_enable.  Driver has no setControlsEnable function")
            self.updateAndPublishStatus(do_updates=False) # Updated inline here

            
    def setAutoAdjustCb(self, msg):
        new_auto_adjust = msg.data
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Auto Adjust request. Controls disabled")
        else:
            if self.setAutoAdjust is None:
                rospy.loginfo("Ignoring Set Auto Adjust. Driver has no setAutoAdjust function")   
            else:
                # Call the parent's method and update ROS param as necessary
                # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                status, err_str = self.setAutoAdjust(new_auto_adjust)
                if status is True:
                    rospy.set_param('~idx/auto_adjust', new_auto_adjust)
                    self.status_msg.auto_adjust = new_auto_adjust
                    if new_auto_adjust:
                        rospy.loginfo("Enabling Auto Adjust")
                    else:
                        rospy.loginfo("Disabling IDX Auto Adjust")
                else:
                    rospy.logerr("Failed to update auto adjust: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here



    def setBrightnessCb(self, msg):
        new_brightness = msg.data
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Brightness request. Controls disabled")
        else:
            if rospy.get_param('~idx/auto', self.init_auto_adjust):
                rospy.loginfo("Ignoring Set Brightness request. Auto Adjust enabled")
            else:
                if self.setBrightness is None:
                    rospy.loginfo("Ignoring Set Brightness. Driver has no setBrightness function")
                else:
                    if (new_brightness < 0.0 or new_brightness > 1.0):
                        rospy.logerr("Brightness value out of bounds")
                    else:
                        # Call the parent's method and update ROS param as necessary
                        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                        status, err_str = self.setBrightness(new_brightness)
                        if status is True:
                            rospy.set_param('~idx/brightness', new_brightness)
                            self.status_msg.brightness = new_brightness
                        else:
                            rospy.logerr("Failed to update brightness: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setContrastCb(self, msg):
        new_contrast = msg.data
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Contrast request. Controls disabled")
        else:
            if rospy.get_param('~idx/auto', self.init_auto_adjust):
                rospy.loginfo("Ignoring Set Contrast request. Auto Adjust enabled")
            else:
                if self.setContrast is None:
                    rospy.loginfo("Ignoring Set Contrast. Driver has no setContrast function")
                else:
                    if (new_contrast < 0.0 and new_contrast != -1.0) or (new_contrast > 1.0):
                        rospy.logerr("Contrast value out of bounds")
                        self.updateAndPublishStatus(do_updates=False) # No change
                        return
                    else:
                        # Call the parent's method and update ROS param as necessary
                        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                        status, err_str = self.setContrast(new_contrast)
                        if status is True:
                            rospy.set_param('~idx/contrast', new_contrast)
                            self.status_msg.contrast = new_contrast
                        else:
                            rospy.logerr("Failed to update contrast: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here


    def setThresholdingCb(self, msg):
        new_thresholding = msg.data
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Thresholding request. Controls disabled")
        else:
            if rospy.get_param('~idx/auto', self.init_auto_adjust):
                rospy.loginfo("Ignoring Set Thresholding request. Auto Adjust enabled")
            else:
                if self.setThresholding is None:
                    rospy.loginfo("Ignoring Set Thresholding. Driver has no setThresholding function")
                else:
                    if (new_thresholding < 0.0 or new_thresholding > 1.0):
                        rospy.logerr("Thresholding value out of bounds")
                        self.updateAndPublishStatus(do_updates=False) # No change
                        return
                    else:
                        # Call the parent's method and update ROS param as necessary
                        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                        status, err_str = self.setThresholding(new_thresholding)
                        if status is True:
                            rospy.set_param('~idx/thresholding', new_thresholding)
                            self.status_msg.thresholding = new_thresholding
                        else:
                            rospy.logerr("Failed to update thresholding: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setResolutionModeCb(self, msg):
        new_resolution = msg.data
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Resolution request. Controls disabled")
        else:
            if self.setResolutionMode is None:
                    rospy.loginfo("Ignoring Set Resolution. Driver has no setResolution function")
            else:
                if (new_resolution > self.RESOLUTION_MODE_MAX):
                        rospy.logerr("Resolution mode value out of bounds")
                        self.updateAndPublishStatus(do_updates=False) # No change
                        return
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setResolutionMode(new_resolution)
                    if status is True:
                        rospy.set_param('~idx/resolution_mode', new_resolution)
                        self.status_msg.resolution_mode = new_resolution
                    else:
                        rospy.logerr("Failed to update resolution: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here


        
    def setFramerateModeCb(self, msg):
        new_framerate = msg.data
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Framerate request. Controls disabled")    
        else: 
            if self.setFramerateMode is None:
                rospy.loginfo("Ignoring Set Framerate. Driver has no setFramerate function")
            else:
                if (new_framerate > self.FRAMERATE_MODE_MAX):
                    rospy.logerr("Framerate mode value out of bounds")
                    self.updateAndPublishStatus(do_updates=False) # No change
                    return
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setFramerateMode(new_framerate)
                    if status is True:
                        rospy.set_param('~idx/framerate_mode', new_framerate)
                        self.status_msg.framerate_mode = new_framerate
                    else:
                        rospy.logerr("Failed to update framerate: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

 
    def setRangeCb(self, msg):
        rospy.loginfo(msg)
        new_start_range_ratio = msg.start_range
        new_stop_range_ratio = msg.stop_range
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Range request. Controls disabled")    
        else: 
            if self.setRange is None:
                rospy.loginfo("Ignoring Set Range. Driver has no setRange function")
            else:
                if (new_start_range_ratio < 0 or new_stop_range_ratio > 1 or new_stop_range_ratio < new_start_range_ratio):
                    rospy.logerr("Range values out of bounds")
                    self.updateAndPublishStatus(do_updates=False) # No change
                    return
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setRange(new_start_range_ratio,new_stop_range_ratio)
                    if status is True:
                        rospy.set_param('~idx/range_window/start_range_ratio', new_start_range_ratio)
                        rospy.set_param('~idx/range_window/stop_range_ratio', new_stop_range_ratio)
                        self.status_msg.range_window.start_range = new_start_range_ratio
                        self.status_msg.range_window.stop_range = new_stop_range_ratio
                    else:
                        rospy.logerr("Failed to update framerate: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here       

    def setZoomCb(self, msg):
        new_zoom = msg.data
        if (new_zoom < 0.0 and new_zoom != -1.0) or (new_zoom > 1.0):
            rospy.logerr("Zoom value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            self.zoom_ratio = new_zoom
            self.status_msg.zoom = new_zoom
            self.render_controls[0] = new_zoom
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setRotateCb(self, msg):
        new_rotate = msg.data
        if (new_rotate < 0.0 and new_rotate != -1.0) or (new_rotate > 1.0):
            rospy.logerr("rotate value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            self.rotate_ratio = new_rotate
            self.status_msg.rotate = new_rotate
            self.render_controls[1] = new_rotate
        self.updateAndPublishStatus(do_updates=False) # Updated inline here  

    def setTiltCb(self, msg):
        new_tilt = msg.data
        if (new_tilt < 0.0 and new_tilt != -1.0) or (new_tilt > 1.0):
            rospy.logerr("tilt value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            self.tilt_ratio = new_tilt
            self.status_msg.tilt = new_tilt
            self.render_controls[2] = new_tilt
        self.updateAndPublishStatus(do_updates=False) # Updated inline here  


    def setFrame3dCb(self, msg):
        new_frame_3d = msg.data
        self.setFrame3d(new_frame_3d)

    def setFrame3d(self, new_frame_3d):
        rospy.set_param('~idx/frame_3d', new_frame_3d)
        self.status_msg.frame_3d = new_frame_3d
        self.updateAndPublishStatus(do_updates=False) # Updated inline here 

    def updateFromParamServer(self):
        param_dict = rospy.get_param('~idx', {})
        #rospy.logwarn("Debugging: param_dict = " + str(param_dict))
        if (self.updateSettings is not None and 'settings' in param_dict):
            self.updateSettings(param_dict['settings'])
        if (self.setControlsEnable is not None and 'controls_enable' in param_dict):
            self.setControlsEnable(param_dict['controls_enable'])
        if (self.setAutoAdjust is not None and 'auto_adjust' in param_dict):
            self.setAutoAdjust(param_dict['auto_adjust'])
        if (self.setBrightness is not None and 'brightness' in param_dict):
            self.setBrightness(param_dict['brightness'])
        if (self.setContrast is not None and 'contrast' in param_dict):
            self.setContrast(param_dict['contrast'])
        if (self.setThresholding is not None and 'thresholding' in param_dict):
            self.setThresholding(param_dict['thresholding'])
        if (self.setResolutionMode is not None and 'resolution_mode' in param_dict):
            self.setResolutionMode(param_dict['resolution_mode'])
        if (self.setFramerateMode is not None and 'framerate_mode' in param_dict):
            self.setFramerateMode(param_dict['framerate_mode'])
        if (self.setRange is not None and 'start_range' in param_dict and 'stop_range' in param_dict):
            self.setRange(param_dict['range_window']['start_range'], param_dict['range_window']['stop_range'])
        self.setFrame3d(param_dict['frame_3d'])
        save_data_configs = rospy.get_param('idx/save_data_configs', self.init_save_data_configs)
        self.updateSaveDataConfigs(save_data_configs)
        save_data_nav_enabled = rospy.get_param('idx/save_data_nav_enabled', self.init_save_data_nav_enabled)
        self.setSaveDataNavEnable(save_data_nav_enabled)
        save_data_enabled = rospy.get_param('idx/save_data_enabled', self.init_save_data_enabled)
        self.setSaveDataEnable(save_data_enabled)


    def provide_capabilities(self, _):
        return self.capabilities_report
    
    def provide_navpose_capabilities(self, _):
        return self.navpose_capabilities_report  

    def setCurrentSettingsAsDefault(self):
        pass # We only use the param server, no member variables to apply to param server
           
    def __init__(self, sensor_name, capSettings=None, 
                 factorySettings=None, settingsUpdateFunction=None, getSettings=None,
                 factoryControls = None, setControlsEnable=None, setAutoAdjust=None,
                 setContrast=None, setBrightness=None, setThresholding=None,
                 setResolutionMode=None, setFramerateMode=None, 
                 setRange=None, 
                 getColor2DImg=None, stopColor2DImgAcquisition=None, 
                 getBW2DImg=None, stopBW2DImgAcquisition=None,
                 getDepthMap=None, stopDepthMapAcquisition=None, 
                 getDepthImg=None, stopDepthImgAcquisition=None, 
                 getPointcloud=None, stopPointcloudAcquisition=None, 
                 getPointcloudImg=None, stopPointcloudImgAcquisition=None, 
                 getGPSMsg=None,getOdomMsg=None,getHeadingMsg=None):
        
        
        self.sensor_name = sensor_name

        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?
        self.cv_bridge = CvBridge()

        self.capabilities_report = IDXCapabilitiesQueryResponse()
        self.navpose_capabilities_report = NavPoseCapabilitiesQueryResponse()

        # Create and update factory settings dictionary
        self.factory_controls = self.FACTORY_CONTROLS
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls[control] = factoryControls[control]


        # Initialize Sensor Settings from Node
        if capSettings is None:
            self.cap_settings = nepi_nex.NONE_SETTINGS
            self.capabilities_report.settings = False
            self.factory_settings = nepi_nex.NONE_SETTINGS
            self.init_settings = rospy.get_param('~idx/settings', nepi_nex.NONE_SETTINGS)
            rospy.set_param('~idx/settings', self.init_settings )
        else:
            self.cap_settings = nepi_nex.sort_settings_alphabetically(capSettings,1)
            self.capabilities_report.settings = True        
            cap_settings_msg = nepi_nex.create_msg_data_from_settings(self.cap_settings)
            self.capabilities_report.settings_options = cap_settings_msg

            if factorySettings is None:
                self.factory_settings = nepi_nex.NONE_SETTINGS
            else:
                self.factory_settings = nepi_nex.sort_settings_alphabetically(factorySettings,1) 
            if settingsUpdateFunction is None:
                self.update_settings_function = None
            else:
                self.update_settings_function = settingsUpdateFunction

            self.getSettings = getSettings
            if self.getSettings is not None:
                current_settings = nepi_nex.sort_settings_alphabetically(self.getSettings(),1) 
                rospy.set_param('~idx/settings', current_settings)
            else:
                current_settings = nepi_nex.NONE_SETTINGS
            # Set init values for resets. Updated saveConfigCb() on save_config msg
            self.init_settings = rospy.get_param('~idx/settings', self.factory_settings)
            rospy.set_param('~idx/settings', self.init_settings )



        self.init_controls_enable = rospy.get_param('~idx/controls_enable',  self.factory_controls["controls_enable"])
        rospy.set_param('~idx/controls_enable', self.init_controls_enable)

        self.init_auto_adjust = rospy.get_param('~idx/auto',  self.factory_controls["auto_adjust"])
        rospy.set_param('~idx/auto', self.init_auto_adjust)

        self.init_brightness_ratio = rospy.get_param('~idx/brightness',  self.factory_controls["brightness_ratio"])
        rospy.set_param('~idx/brightness', self.init_brightness_ratio)

        self.init_contrast_ratio = rospy.get_param('~idx/contrast',  self.factory_controls["contrast_ratio"])
        rospy.set_param('~idx/contrast', self.init_contrast_ratio)

        self.init_threshold_ratio = rospy.get_param('~idx/thresholding',  self.factory_controls["threshold_ratio"])
        rospy.set_param('~idx/thresholding', self.init_threshold_ratio) 

        self.init_resolution_mode = rospy.get_param('~idx/resolution_mode',  self.factory_controls["resolution_mode"])
        rospy.set_param('~idx/resolution_mode', self.init_resolution_mode)

        self.init_framerate_mode = rospy.get_param('~idx/framerate_mode',  self.factory_controls["framerate_mode"])
        rospy.set_param('~idx/framerate_mode', self.init_framerate_mode)

        if self.factory_controls["frame_3d"] is None:
            self.init_frame_3d = rospy.get_param('~idx/frame_3d',  'nepi_center_frame')
        else:
            self.init_frame_3d = rospy.get_param('~idx/frame_3d',  self.factory_controls["frame_3d"] )
        rospy.set_param('~idx/frame_3d', self.init_frame_3d)
        

        self.init_start_range_ratio = rospy.get_param('~idx/range_window/start_range_ratio',  self.factory_controls["start_range_ratio"])
        rospy.set_param('~idx/range_window/start_range_ratio', self.init_start_range_ratio)
        self.init_stop_range_ratio = rospy.get_param('~idx/range_window/stop_range_ratio', self.factory_controls["stop_range_ratio"])
        rospy.set_param('~idx/range_window/stop_range_ratio', self.init_stop_range_ratio)
        self.init_min_range_m = rospy.get_param('~idx/range_limits/min_range_m',  self.factory_controls["min_range_m"])
        rospy.set_param('~idx/range_limits/min_range_m', self.init_min_range_m)
        self.init_max_range_m = rospy.get_param('~idx/range_limits/max_range_m',  self.factory_controls["max_range_m"])
        rospy.set_param('~idx/range_limits/max_range_m', self.init_max_range_m)

        # Set up standard IDX parameters with ROS param and subscriptions
        # Defer actually setting these on the camera via the parent callbacks... the parent may need to do some 
        # additional setup/calculation first. Parent can then get these all applied by calling updateFromParamServer()


        
        #************************************ new settings testing        
        self.setControlsEnable = setControlsEnable
        if setControlsEnable is not None:
            rospy.Subscriber('~idx/set_controls_enable', Bool, self.setControlsEnableCb, queue_size=1) # start local callback
     
        self.setAutoAdjust = setAutoAdjust
        if setAutoAdjust is not None:
            rospy.Subscriber('~idx/set_auto_adjust', Bool, self.setAutoAdjustCb, queue_size=1) # start local callback
            self.capabilities_report.auto_adjustment = True
        else:
            self.capabilities_report.auto_adjustment = False
    
        self.setBrightness = setBrightness
        if setBrightness is not None:
            rospy.Subscriber('~idx/set_brightness', Float32, self.setBrightnessCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_brightness = True
        else:
            self.capabilities_report.adjustable_brightness = False

        self.setContrast = setContrast
        if setContrast is not None:
            rospy.Subscriber('~idx/set_contrast', Float32, self.setContrastCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_contrast = True
        else:
            self.capabilities_report.adjustable_contrast = False

        self.setThresholding = setThresholding       
        if setThresholding is not None:
            rospy.Subscriber('~idx/set_thresholding', Float32, self.setThresholdingCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_thresholding = True
        else:
            self.capabilities_report.adjustable_thresholding = False

        self.setResolutionMode = setResolutionMode
        if setResolutionMode is not None:
            rospy.Subscriber('~idx/set_resolution_mode', UInt8, self.setResolutionModeCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_resolution = True
        else:
            self.capabilities_report.adjustable_resolution = False
               
        self.setFramerateMode = setFramerateMode
        if setFramerateMode is not None:
            rospy.Subscriber('~idx/set_framerate_mode', UInt8, self.setFramerateModeCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_framerate = True
        else:
            self.capabilities_report.adjustable_framerate = False

        self.setRange = setRange
        if setRange is not None:        
            rospy.Subscriber('~idx/set_range_window', RangeWindow, self.setRangeCb, queue_size=1)
            self.capabilities_report.adjustable_range = True
        else:
            self.capabilities_report.adjustable_range = False


        rospy.Subscriber('~idx/set_frame_3d', String, self.setFrame3dCb, queue_size=1)

        rospy.Subscriber('~idx/update_settings', String, self.updateSettingsCb, queue_size=1) # start local callbac
        rospy.Subscriber('~idx/reset_controls', Empty, self.resetControlsCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/reset_settings', Empty, self.resetSettingsCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/reset_factory', Empty, self.resetFactoryCb, queue_size=1) # start local callback

        rospy.Subscriber('~idx/update_save_data_configs', String, self.updateSaveDataConfigsCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/set_save_data_enable', Bool, self.setSaveDataEnableCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/set_save_data_nav_enable', Bool, self.setSaveDataNavEnableCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/set_save_data_prefix', String, self.setSaveDataPrefixCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/reset_save_data', Empty, self.resetSaveDataCb, queue_size=1) # start local callback

        # Create Nav Data Saving Publishers
        nepi_base_namespace = nepi_ros.get_base_namespace()
        self.nav_data_product = "nav_pose"
        save_nav_prefix_topic = nepi_base_namespace + "nav_pose_mgr/save_data_prefix"
        save_nav_rate_topic = nepi_base_namespace + "nav_pose_mgr/save_data_rate"
        save_nav_enable_topic = nepi_base_namespace + "nav_pose_mgr/save_data"

        rospy.loginfo("Save Nav topic: " + save_nav_rate_topic)

        self.set_nav_save_prefix_pub = rospy.Publisher(save_nav_prefix_topic, String, queue_size=10)
        self.set_nav_save_rate_pub = rospy.Publisher(save_nav_rate_topic, SaveDataRate, queue_size=10)
        self.set_nav_save_enable_pub = rospy.Publisher(save_nav_enable_topic, SaveData, queue_size=10)

        # Start the data producers  
        self.getColor2DImg = getColor2DImg
        if (self.getColor2DImg is not None):
            self.color_img_pub = rospy.Publisher('~idx/color_2d_image', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('color_2d_image')
            self.color_img_thread = threading.Thread(target=self.runColorImgThread)
            self.color_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopColor2DImgAcquisition = stopColor2DImgAcquisition
            self.capabilities_report.has_color_2d_image = True
        else:
            self.capabilities_report.has_color_2d_image = False
        

        self.getBW2DImg = getBW2DImg
        if (self.getBW2DImg is not None):
            self.bw_img_pub = rospy.Publisher('~idx/bw_2d_image', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('bw_2d_image')
            self.bw_img_thread = threading.Thread(target=self.runBWImgThread)
            self.bw_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopBW2DImgAcquisition = stopBW2DImgAcquisition
            self.capabilities_report.has_bw_2d_image = True
        else:
            self.capabilities_report.has_bw_2d_image = False
        

        self.getDepthMap = getDepthMap
        if (self.getDepthMap is not None):
            self.depth_map_pub = rospy.Publisher('~idx/depth_map', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('depth_map')
            self.depth_map_thread = threading.Thread(target=self.runDepthMapThread)
            self.depth_map_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopDepthMapAcquisition = stopDepthMapAcquisition
            self.capabilities_report.has_depth_map = True
        else:
            self.capabilities_report.has_depth_map = False
            
        self.getDepthImg = getDepthImg
        if (self.getDepthImg is not None):
            self.depth_img_pub = rospy.Publisher('~idx/depth_image', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('depth_image')
            self.depth_img_thread = threading.Thread(target=self.runDepthImgThread)
            self.depth_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopDepthImgAcquisition = stopDepthImgAcquisition
            self.capabilities_report.has_depth_image = True
        else:
            self.capabilities_report.has_depth_image = False

        self.getPointcloud = getPointcloud
        if (self.getPointcloud is not None):
            self.pointcloud_pub = rospy.Publisher('~idx/pointcloud', PointCloud2, queue_size=1, tcp_nodelay=True)
            self.data_products.append('pointcloud')
            self.pointcloud_thread = threading.Thread(target=self.runPointcloudThread)
            self.pointcloud_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopPointcloudAcquisition = stopPointcloudAcquisition
            self.capabilities_report.has_pointcloud = True
        else:
            self.capabilities_report.has_pointcloud = False

        self.getPointcloudImg = getPointcloudImg
        if (self.getPointcloudImg is not None):
            self.pointcloud_img_pub = rospy.Publisher('~idx/pointcloud_image', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('pointcloud_image')
            self.pointcloud_img_thread = threading.Thread(target=self.runPointcloudImgThread)
            self.pointcloud_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopPointcloudImgAcquisition = stopPointcloudImgAcquisition
            self.capabilities_report.has_pointcloud_image = True

            rospy.Subscriber('~idx/set_zoom', Float32, self.setZoomCb, queue_size=1) # start local callback
            rospy.Subscriber('~idx/set_rotate', Float32, self.setRotateCb, queue_size=1) # start local callback
            rospy.Subscriber('~idx/set_tilt', Float32, self.setTiltCb, queue_size=1) # start local callback
        else:
            self.capabilities_report.has_pointcloud_image = False
        
        self.getGPSMsg = getGPSMsg
        if getGPSMsg is not None:
            self.idx_navpose_gps_pub = rospy.Publisher('~idx/gps_fix', NavSatFix, queue_size=1)
            self.navpose_capabilities_report.has_gps = True
        else:
            self.navpose_capabilities_report.has_gps = False

        self.getOdomMsg = getOdomMsg
        if getOdomMsg is not None:
            self.idx_navpose_odom_pub = rospy.Publisher('~idx/odom', Odometry, queue_size=1)
            self.navpose_capabilities_report.has_orientation = True
        else:
            self.navpose_capabilities_report.has_orientation = False

        self.getHeadingMsg = getHeadingMsg
        if getHeadingMsg is not None:
            self.idx_navpose_heading_pub = rospy.Publisher('~idx/heading', Float32, queue_size=1)
            self.navpose_capabilities_report.has_heading = True
        else:
            self.navpose_capabilities_report.has_heading = False


        # Set up the save data and save cfg i/f and launch saving threads-- Do this before launching aquisition threads so that they can check data_product_should_save() immediately
        self.capabilities_report.data_products = str(self.data_products)

        self.save_data_if = SaveDataIF(data_product_names = self.data_products)

        # Update save data configuration fom param server

        for data_product in self.data_products:
            self.factory_save_data_configs.append([data_product,self.FACTORY_SAVE_DATA_CONFIGS.get(data_product)])
        self.init_save_data_configs = rospy.get_param('~idx/save_data_configs',  self.factory_save_data_configs)
        rospy.set_param('~idx/save_data_configs', self.init_save_data_configs) 
        self.init_save_data_enabled = rospy.get_param('~idx/save_data_enabled',  self.factory_save_data_enabled)
        rospy.set_param('~idx/save_data_enabled', self.init_save_data_enabled) 
        self.init_save_data_nav_enabled = rospy.get_param('~idx/save_data_nav_enabled',  self.factory_save_data_nav_enabled)
        rospy.set_param('~idx/save_data_nav_enabled', self.init_save_data_nav_enabled) 


        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer)

    
        # Set up additional publishers
        self.status_msg = IDXStatus()
        self.status_pub = rospy.Publisher('~idx/status', IDXStatus, queue_size=1, latch=True)

        if getGPSMsg != None or getOdomMsg != None or getHeadingMsg != None:
            rospy.Timer(rospy.Duration(self.update_navpose_interval_sec), self.navposeCb)


        self.updateAndPublishStatus()


        # Start capabilities services
        rospy.Service('~idx/capabilities_query', IDXCapabilitiesQuery, self.provide_capabilities)
        rospy.Service('~idx/navpose_capabilities_query', NavPoseCapabilitiesQuery, self.provide_navpose_capabilities)
        
        # Launch the acquisition and saving threads
        if (self.getColor2DImg is not None):
            self.color_img_thread.start()
            self.color_2d_image = None
            self.color_2d_image_timestamp = None
            self.color_2d_image_lock = threading.Lock()
            rospy.Timer(rospy.Duration(self.check_data_save_interval_sec), self.saveColorImgThread)

        if (self.getBW2DImg is not None):
            self.bw_img_thread.start()
            self.bw_2d_image = None
            self.bw_2d_image_timestamp = None
            self.bw_2d_image_lock = threading.Lock()
            rospy.Timer(rospy.Duration(self.check_data_save_interval_sec), self.saveBWImgThread)

        if (self.getDepthMap is not None):
            self.depth_map_thread.start()
            self.depth_map = None
            self.depth_map_timestamp = None
            self.depth_map_lock = threading.Lock()
            rospy.Timer(rospy.Duration(self.check_data_save_interval_sec), self.saveDepthMapThread)
        
        if (self.getDepthImg is not None):
            self.depth_img_thread.start()
            self.depth_image = None
            self.depth_image_timestamp = None
            self.depth_image_lock = threading.Lock()
            rospy.Timer(rospy.Duration(self.check_data_save_interval_sec), self.saveDepthImgThread)
        
        if (self.getPointcloud is not None):
            self.pointcloud_thread.start()
            self.pointcloud = None
            self.pointcloud_timestamp = None
            self.pointcloud_lock = threading.Lock()
            rospy.Timer(rospy.Duration(self.check_data_save_interval_sec), self.savePointcloudThread)

        if (self.getPointcloudImg is not None):
            self.pointcloud_img_thread.start()
            self.pointcloud_image = None
            self.pointcloud_image_timestamp = None
            self.pointcloud_image_lock = threading.Lock()
            rospy.Timer(rospy.Duration(self.check_data_save_interval_sec), self.savePointcloudImgThread)
        
        # Start a regular check for save status changes
        rospy.Timer(rospy.Duration(0.5), self.updateSaveDataStatusCallback)
        
    # Image from img_get_function can be CV2 or ROS image.  Will be converted as needed in the thread
    def image_thread_proccess(self,data_product,img_get_function,img_stop_function,img_publisher):
        image = None
        cv2_img = None
        ros_img = None
        if not rospy.is_shutdown():
            rospy.loginfo(rospy.get_name() + ": starting " + data_product + " acquisition thread")
            acquiring = False
            while (not rospy.is_shutdown()):
                saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
                has_subscribers = (img_publisher.get_num_connections() > 0)
                if (has_subscribers is True) or (saving_is_enabled is True):
                    acquiring = True
                    if data_product != "pointcloud_image":
                        status, msg, image, ros_timestamp, encoding = img_get_function()
                    else:
                        status, msg, image, ros_timestamp, encoding = img_get_function(self.render_controls)
                    if (status is False):
                        #rospy.logerr_throttle(1, msg)
                        continue   
                    if (has_subscribers is True):
                        if isinstance(image,np.ndarray):  # CV2 image. Convert to ROS Image   
                            # Convert cv to ros   
                            ros_img = nepi_img.cv2img_to_rosimg(image, encoding=encoding)
                            ros_img.header.stamp = ros_timestamp
                            ros_img.header.frame_id = rospy.get_param('~idx/frame_3d',  self.init_frame_3d )
                        elif isinstance(image,Image): # ROS Image. Passthrough
                            ros_img = image
                        if ros_img is not None:
                            # Publish image
                            img_publisher.publish(ros_img)
                    if (saving_is_enabled is True):
                        if isinstance(image,np.ndarray):  # CV2 image. Passthrough   
                            cv2_img = image
                        elif isinstance(image,Image): # ROS Image. Convert to CV2 Image
                            cv2_img = nepi_img.rosimg_to_cv2img(image, encoding=encoding)
                        if eval("self." + data_product + "_lock.locked() is False"):
                            eval("self." + data_product + "_lock.acquire()")
                            exec("self." + data_product + " = cv2_img")
                            exec("self." + data_product + "_timestamp = ros_timestamp")
                            eval("self." + data_product + "_lock.release()")
                elif acquiring is True:
                    if img_stop_function is not None:
                        rospy.loginfo("Stopping " + data_product + " acquisition")
                        img_stop_function()
                    acquiring = False
                else: # No subscribers and already stopped
                    acquiring = False
                    rospy.sleep(0.25)
                rospy.sleep(0.01) # Yield



    
    # Pointcloud from pointcloud_get_function can be open3D or ROS pointcloud.  Will be converted as needed in the thread
    def pointcloud_thread_proccess(self,data_product,pc_get_function,pc_stop_function,pc_publisher):
        pc = None
        o3d_pc = None
        ros_pc = None
        if not rospy.is_shutdown():
            rospy.loginfo(rospy.get_name() + ": starting " + data_product + " acquisition thread")
            acquiring = False
            while (not rospy.is_shutdown()):
                saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
                has_subscribers = (pc_publisher.get_num_connections() > 0)
                if (has_subscribers is True) or (saving_is_enabled is True):
                    acquiring = True
                    status, msg, pc, ros_timestamp, ros_frame = pc_get_function()
                    #********************
                    frame_id = rospy.get_param('~idx/frame_3d',  self.init_frame_3d )
                    if frame_id == 'sensor_frame': # passthrough from sensor
                        pass
                    else: # replace with idx selected frame
                        ros_frame = rospy.get_param('~idx/frame_3d',  self.init_frame_3d )
                    #********************
                    if (status is False):
                        #rospy.logerr_throttle(1, msg)
                        continue   
                    if (has_subscribers is True):
                        if isinstance(pc,o3d.geometry.PointCloud):  # Open3D Pointcloud. Convert to ROS Pointcloud   
                            # Convert o3d to ros   
                            ros_pc = nepi_pc.o3dpc_to_rospc(pc, stamp = ros_timestamp, frame_id = ros_frame )
                        elif isinstance(pc,PointCloud2): # ROS Pointcloud. Passthrough
                            ros_pc = pc
                        if ros_pc is not None:
                            # Publish Pointcloud
                            pc_publisher.publish(ros_pc)
                    if (saving_is_enabled is True):
                        if isinstance(pc,o3d.geometry.PointCloud):  # Open3d pointcloud. Passthrough   
                            o3d_pc = pc
                        elif isinstance(pc,PointCloud2): # ROS Pointcloud. Convert to Open3d pointcloud
                            o3d_pc = nepi_pc.rospc_to_o3dpc(pc)
                        if eval("self." + data_product + "_lock.locked() is False"):
                            eval("self." + data_product + "_lock.acquire()")
                            exec("self." + data_product + " = o3d_pc")
                            exec("self." + data_product + "_timestamp = ros_timestamp")
                            eval("self." + data_product + "_lock.release()")
                elif acquiring is True:
                    if pc_stop_function is not None:
                        rospy.loginfo("Stopping " + data_product + " acquisition")
                        pc_stop_function()
                    acquiring = False
                else: # No subscribers and already stopped
                    acquiring = False
                    rospy.sleep(0.25)
                rospy.sleep(0.01) # Yield
                

    def runColorImgThread(self):
        self.image_thread_proccess('color_2d_image', self.getColor2DImg, self.stopColor2DImgAcquisition, self.color_img_pub)
        
    def runBWImgThread(self):
        self.image_thread_proccess('bw_2d_image', self.getBW2DImg, self.stopBW2DImgAcquisition, self.bw_img_pub)

    def runDepthMapThread(self):
        self.image_thread_proccess('depth_map', self.getDepthMap, self.stopDepthMapAcquisition, self.depth_map_pub)

    def runDepthImgThread(self):
        self.image_thread_proccess('depth_image', self.getDepthImg, self.stopDepthImgAcquisition, self.depth_img_pub)

    def runPointcloudThread(self):
        self.pointcloud_thread_proccess('pointcloud', self.getPointcloud, self.stopPointcloudAcquisition, self.pointcloud_pub)

    def runPointcloudImgThread(self):
        self.image_thread_proccess('pointcloud_image', self.getPointcloudImg, self.stopPointcloudImgAcquisition, self.pointcloud_img_pub)



# Define saving functions for saving callbacks



    def save_img2file(self,data_product,cv2_image,ros_timestamp):
        if self.save_data_if is not None:
            saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
            # Save data if enabled
            if saving_is_enabled:
                eval("self." + data_product + "_lock.acquire()")
                if eval("self." + data_product + " is not None"):
                    if (self.save_data_if.data_product_should_save(data_product) is True):
                        full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                                self.sensor_name + "-" + data_product, 'png')
                        if cv2_image is not None and os.path.isfile(full_path_filename) is False:
                            cv2.imwrite(full_path_filename, cv2_image)
                eval("self." + data_product + "_lock.release()")

    def save_pc2file(self,data_product,o3d_pc,ros_timestamp):
        if self.save_data_if is not None:
            saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
            if saving_is_enabled:
                eval("self." + data_product + "_lock.acquire()")
                if eval("self." + data_product + " is not None"):
                    if (self.save_data_if.data_product_should_save(data_product) is True):
                        full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                                self.sensor_name + "-" + data_product, 'pcd')
                        if o3d_pc is not None and os.path.isfile(full_path_filename) is False:
                            nepi_pc.save_pointcloud(o3d_pc,full_path_filename)
                eval("self." + data_product + "_lock.release()")
                

    # Create timer callbacks for saving threads
    def updateSaveDataStatusCallback(self,timer):
        changed = False
        # Check current data product save configs for change
        configs_if = self.save_data_if.getSaveDataConfigs()
        configs = rospy.get_param('~idx/save_data_configs',  self.init_save_data_configs)
        if configs != configs_if:
            rospy.set_param('~idx/save_data_configs', configs_if)
            self.status_msg.save_data_configs = nepi_nex.create_save_configs_msg(configs_if)
            changed = True
        if self.save_data_prefix != self.save_data_if.save_data_prefix:
            self.save_data_prefix = self.save_data_if.save_data_prefix
            self.status_msg.save_data_prefix = self.save_data_prefix
            changed = True
        # Check current data save enabled for change
        save_enabled_if = (self.save_data_if.save_continuous or self.save_data_if.save_raw)
        save_enabled = rospy.get_param('~idx/save_data_enabled',  self.init_save_data_enabled)
        if save_enabled != save_enabled_if:
            rospy.set_param('~idx/save_data_enabled', save_enabled_if)
            self.status_msg.save_data_enabled = save_enabled_if
            changed = True
        if changed:
            self.updateAndPublishStatus(do_updates = False) # Status manually updated above

    def saveColorImgThread(self,timer):
        data_product = 'color_2d_image'
        eval("self.save_img2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")
                
    def saveBWImgThread(self,timer):
        data_product = 'bw_2d_image'
        eval("self.save_img2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")

    def saveDepthMapThread(self,timer):
        data_product = 'depth_map'
        eval("self.save_img2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")

    def saveDepthImgThread(self,timer):
        data_product = 'depth_image'
        eval("self.save_img2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")

    def savePointcloudThread(self,timer):
        data_product = 'pointcloud'
        eval("self.save_pc2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")

    def savePointcloudImgThread(self,timer):
        data_product = 'pointcloud_image'
        eval("self.save_img2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")

    def navposeCb(self, timer):
        if self.getGPSMsg != None:
            gps_msg = self.getGPSMsg()
            if gps_msg is not None:
                self.idx_navpose_gps_pub.publish(gps_msg)

        if self.getOdomMsg != None:
            odom_msg = self.getOdomMsg()
            if odom_msg is not None:
                self.idx_navpose_odom_pub.publish(odom_msg)

        if self.getHeadingMsg != None:
            heading_msg = self.getHeadingMsg()
            if heading_msg is not None:
                self.idx_navpose_gps_pub.publish(heading_msg)   


    def getCurrentSettings(self):
        if self.getSettings is not None:
            current_settings = self.getSettings()
            current_settings= nepi_nex.sort_settings_alphabetically(current_settings,1)
            if current_settings is None:
                current_settings = nepi_nex.NONE_SETTINGS
        else:
            current_settings = nepi_nex.NONE_SETTINGS
        return current_settings



    # Function to update and publish status message

    def updateAndPublishStatus(self, do_updates = True):
        if do_updates is True:
            # TODO: Probably these should be queried from the parent (and through the driver) via explicit callbacks rather than via the param server
            idx_params = rospy.get_param('~idx')
            if self.getSettings is not None:
                current_settings = nepi_nex.sort_settings_alphabetically(self.getSettings(),1) 
                rospy.set_param('~idx/settings', current_settings)
            else:
                current_settings = nepi_nex.NONE_SETTINGS
            settings_msg = nepi_nex.create_msg_data_from_settings(current_settings)
            self.status_msg.settings = settings_msg 
            self.status_msg.controls_enable = idx_params['controls_enable'] if 'controls_enable' in idx_params else True
            self.status_msg.auto_adjust = idx_params['auto_adjust'] if 'auto_adjust' in idx_params else False
            self.status_msg.resolution_mode = idx_params['resolution_mode'] if 'resolution_mode' in idx_params else 0
            self.status_msg.framerate_mode = idx_params['framerate_mode'] if 'framerate_mode' in idx_params else 0
            self.status_msg.contrast = idx_params['contrast'] if 'contrast' in idx_params else 0
            self.status_msg.brightness = idx_params['brightness'] if 'brightness' in idx_params else 0
            self.status_msg.thresholding = idx_params['thresholding'] if 'thresholding' in idx_params else 0
            
            self.status_msg.range_window.start_range = rospy.get_param('~idx/range_window/start_range_ratio', 0.0)
            self.status_msg.range_window.stop_range =  rospy.get_param('~idx/range_window/stop_range_ratio', 1.0)
            self.status_msg.min_range_m = rospy.get_param('~idx/range_limits/min_range_m',0.0)
            self.status_msg.max_range_m = rospy.get_param('~idx/range_limits/max_range_m',1.0)
            # The transfer frame into which 3D data (pointclouds) are transformed for the pointcloud data topic
            self.status_msg.frame_3d = idx_params['frame_3d'] if 'frame_3d' in idx_params else "nepi_center_frame"
            save_data_configs = idx_params['save_data_configs'] if 'save_data_configs' in idx_params else []
            self.status_msg.save_data_configs = nepi_nex.create_save_configs_msg(save_data_configs)
            self.status_msg.save_data_prefix = self.save_data_prefix
            self.status_msg.save_data_enabled = idx_params['save_data_enabled'] if 'save_data_enabled' in idx_params else False
            self.status_msg.save_data_nav_enabled = idx_params['save_data_nav_enabled'] if 'save_data_nav_enabled' in idx_params else False
            self.status_msg.zoom = self.zoom_ratio
            self.status_msg.rotate = self.rotate_ratio
            self.status_msg.tilt = self.tilt_ratio

        self.status_pub.publish(self.status_msg)
    
