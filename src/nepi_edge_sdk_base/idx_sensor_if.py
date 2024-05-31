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
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow
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
    CHECK_DATA_SAVE_RATE_HZ = 1
    
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
        zoom_ratio = 0.5, 
        rotate_ratio = 0.5,
        frame_3d = 'nepi_center_frame'
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
    init_zoom_ratio = None
    inti_zoom_ratio = None


    caps_settings = None
    factory_settings = None
    init_settings = None
    update_settings_function = None
    data_products = []

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
        
        rospy.set_param('~idx/zoom', self.factory_controls.get('zoom_ratio'))
        self.status_msg.zoom = self.factory_controls.get('zoom_ratio')

        rospy.set_param('~idx/rotate', self.factory_controls.get('rotate_ratio'))
        self.status_msg.rotate = self.factory_controls.get('rotate_ratio')

        rospy.set_param('~idx/frame_3d', self.factory_controls.get('frame_3d'))
        self.status_msg.frame_3d = self.factory_controls.get('frame_3d')

        self.updateSettings(self.factory_settings)

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
        self.init_zoom_ratio = rospy.get_param('~idx/zoom', self.factory_controls.get('zoom_ratio'))
        self.init_rotate_ratio = rospy.get_param('~idx/rotate', self.factory_controls.get('rotate_ratio'))
        self.init_frame_3d = rospy.get_param('~idx/frame_3d', self.factory_controls.get('frame_3d'))
        self.init_settings = rospy.get_param('~idx/settings',self.factory_settings)
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
        
        rospy.set_param('~idx/zoom', self.init_zoom_ratio)
        self.status_msg.zoom = self.init_zoom_ratio
    
        rospy.set_param('~idx/rotate', self.init_rotate_ratio)
        self.status_msg.rotate = self.init_rotate_ratio

        rospy.set_param('~idx/frame_3d', self.init_frame_3d)
        self.status_msg.frame_3d = self.init_frame_3d

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
                        [current_settings,success,msg] = nepi_nex.try_to_update_setting(setting,current_settings,self.cap_settings,self.update_settings_function)
                        rospy.loginfo(msg)
                    else:
                        rospy.loginfo("Skipping setting update " + str(setting) + " because it matches current setting")
                rospy.set_param('~idx/settings', current_settings)
                current_settings = nepi_nex.sort_settings_alphabetically(current_settings,1)
            else:
                rospy.loginfo("Settings updates ignored. No settings update function defined ")
            settings_msg = nepi_nex.create_msg_data_from_settings(current_settings)
            self.status_msg.settings = settings_msg 
            self.updateAndPublishStatus(do_updates=False) # Updated inline here
        else:
            rospy.loginfo("Skipping settings update request. getCurrentSettings function not defined")
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
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Zoom request. Controls disabled")
        else:
            if self.setZoom is None:
                rospy.loginfo("Ignoring Set Zoom. Driver has no setZoom function")
            else:
                if (new_zoom < 0.0 and new_zoom != -1.0) or (new_zoom > 1.0):
                    rospy.logerr("Zoom value out of bounds")
                    self.updateAndPublishStatus(do_updates=False) # No change
                    return
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setZoom(new_zoom)
                    if status is True:
                        rospy.set_param('~idx/zoom', new_zoom)
                        self.status_msg.zoom = new_zoom
                    else:
                        rospy.logerr("Failed to update Zoom: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setRotateCb(self, msg):
        new_rotate = msg.data
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Rotate request. Controls disabled")
        else:
            if self.setRotate is None:
                rospy.loginfo("Ignoring Set Rotate. Driver has no setRotate function")
            else:
                if (new_rotate < 0.0 and new_rotate != -1.0) or (new_rotate > 1.0):
                    rospy.logerr("Rotate value out of bounds")
                    self.updateAndPublishStatus(do_updates=False) # No change
                    return
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setRotate(new_rotate)
                    if status is True:
                        rospy.set_param('~idx/rotate', new_rotate)
                        self.status_msg.rotate = new_rotate
                    else:
                        rospy.logerr("Failed to update Rotate: " + err_str)
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
        if (self.setAutoAdjust is not None and 'auto' in param_dict):
            self.setAutoAdjust(param_dict['auto'])
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
        if (self.setZoom is not None and 'zoom' in param_dict):
            self.setZoom(param_dict['zoom'])
        if (self.setRotate is not None and 'rotate' in param_dict):
            self.setRotate(param_dict['rotate'])
        self.setFrame3d(param_dict['frame_3d'])

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
                 setRange=None, setZoom=None, setRotate=None,
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
            self.factory_settings = nepi_nex.NONE_SETTINGS
            self.capabilities_report.settings = False
        else:
            self.cap_settings = capSettings
            self.capabilities_report.settings = True
            if factorySettings is None:
                self.factory_settings = nepi_nex.NONE_SETTINGS
            else:
                self.factory_settings = factorySettings   
            if settingsUpdateFunction is None:
                self.update_settings_function = None
            else:
                self.update_settings_function = settingsUpdateFunction
        self.getSettings = getSettings

        self.cap_settings = nepi_nex.sort_settings_alphabetically(self.cap_settings,1)
        self.factory_settings= nepi_nex.sort_settings_alphabetically(self.factory_settings,1)

        cap_settings_msg = nepi_nex.create_msg_data_from_settings(self.cap_settings)
        self.capabilities_report.settings_options = cap_settings_msg

        

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

        self.init_zoom_ratio = rospy.get_param('~idx/zoom',  self.factory_controls["zoom_ratio"])
        rospy.set_param('~idx/zoom', self.init_zoom_ratio) 

        self.init_rotate_ratio = rospy.get_param('~idx/rotate',  self.factory_controls["rotate_ratio"])
        rospy.set_param('~idx/rotate', self.init_rotate_ratio) 

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

        self.setZoom = setZoom      
        if setZoom is not None:
            rospy.Subscriber('~idx/set_zoom', Float32, self.setZoomCb, queue_size=1) # start local callback
            self.capabilities_report.zoom = True
        else:
            self.capabilities_report.zoom = False

        self.setRotate = setRotate       
        if setRotate is not None:
            rospy.Subscriber('~idx/set_rotate', Float32, self.setRotateCb, queue_size=1) # start local callback
            self.capabilities_report.rotate = True
        else:
            self.capabilities_report.rotate = False

        rospy.Subscriber('~idx/set_frame_3d', String, self.setFrame3dCb, queue_size=1)

        rospy.Subscriber('~idx/update_settings', String, self.updateSettingsCb, queue_size=1) # start local callbac
        rospy.Subscriber('~idx/reset_controls', Empty, self.resetControlsCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/reset_settings', Empty, self.resetSettingsCb, queue_size=1) # start local callback
        rospy.Subscriber('~idx/reset_factory', Empty, self.resetFactoryCb, queue_size=1) # start local callback
        rospy.Subscriber('~save_config', Empty, self.saveConfigCb, queue_size=1) # start local callback
 

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
        

        self.getPointcloudImg = getPointcloudImg
        if (self.getPointcloudImg is not None):
            self.pointcloud_img_pub = rospy.Publisher('~idx/pointcloud_image', Image, queue_size=1, tcp_nodelay=True)
            self.data_products.append('pointcloud_image')
            self.pointcloud_img_thread = threading.Thread(target=self.runPointcloudImgThread)
            self.pointcloud_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopPointcloudImgAcquisition = stopPointcloudImgAcquisition 
            self.capabilities_report.has_pointcloud_image = True
        else:
            self.capabilities_report.has_pointcloud = False
               
         

        self.getPointcloud = getPointcloud
        if (self.getPointcloud is not None):
            self.pointcloud_pub = rospy.Publisher('~idx/pointcloud', PointCloud2, queue_size=1, tcp_nodelay=True)
            self.data_products.append('pointcloud')
            self.pointcloud_thread = threading.Thread(target=self.runPointcloudThread)
            self.pointcloud_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.stopPointcloudAcquisition = stopPointcloudAcquisition
            self.capabilities_report.has_pointcloud = True
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
        
        if (self.getPointcloudImg is not None):
            self.pointcloud_img_thread.start()
            self.pointcloud_image = None
            self.pointcloud_image_timestamp = None
            self.pointcloud_image_lock = threading.Lock()
            rospy.Timer(rospy.Duration(self.check_data_save_interval_sec), self.savePointcloudImgThread)

        if (self.getPointcloud is not None):
            self.pointcloud_thread.start()
            self.pointcloud = None
            self.pointcloud_timestamp = None
            self.pointcloud_lock = threading.Lock()            
            rospy.Timer(rospy.Duration(self.check_data_save_interval_sec), self.savePointcloudThread)
        
    
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
                    status, msg, image, ros_timestamp, encoding = img_get_function()
                    if (status is False):
                        rospy.logerr_throttle(1, msg)
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
                        rospy.logerr_throttle(1, msg)
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

    def runPointcloudImgThread(self):
        self.image_thread_proccess('pointcloud_image', self.getPointcloudImg, self.stopPointcloudImgAcquisition, self.pointcloud_img_pub)

    def runPointcloudThread(self):
        self.pointcloud_thread_proccess('pointcloud', self.getPointcloud, self.stopPointcloudAcquisition, self.pointcloud_pub)



# Define saving functions for saving callbacks

    def save_img2file(self,data_product,cv2_image,ros_timestamp):
        if self.save_data_if is not None:
            saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
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

    def savePointcloudImgThread(self,timer):
        data_product = 'pointcloud_image'
        eval("self.save_img2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")

    def savePointcloudThread(self,timer):
        data_product = 'pointcloud'
        eval("self.save_pc2file(data_product,self." + data_product + ",self." + data_product + "_timestamp)")

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
            settings_msg = nepi_nex.create_msg_data_from_settings(idx_params['settings'] if 'settings' in idx_params else self.getCurrentSettings())
            self.status_msg.settings = settings_msg 
            self.status_msg.controls_enable = idx_params['controls_enable'] if 'controls_enable' in idx_params else True
            self.status_msg.auto_adjust = idx_params['auto'] if 'auto' in idx_params else False
            self.status_msg.resolution_mode = idx_params['resolution_mode'] if 'resolution_mode' in idx_params else 0
            self.status_msg.framerate_mode = idx_params['framerate_mode'] if 'framerate_mode' in idx_params else 0
            self.status_msg.contrast = idx_params['contrast'] if 'contrast' in idx_params else 0
            self.status_msg.brightness = idx_params['brightness'] if 'brightness' in idx_params else 0
            self.status_msg.thresholding = idx_params['thresholding'] if 'thresholding' in idx_params else 0
            
            self.status_msg.range_window.start_range = rospy.get_param('~idx/range_window/start_range_ratio', 0.0)
            self.status_msg.range_window.stop_range =  rospy.get_param('~idx/range_window/stop_range_ratio', 1.0)
            self.status_msg.min_range_m = rospy.get_param('~idx/range_limits/min_range_m',0.0)
            self.status_msg.max_range_m = rospy.get_param('~idx/range_limits/max_range_m',1.0)
            self.status_msg.zoom = idx_params['zoom'] if 'zoom' in idx_params else 0.5
            self.status_msg.rotate = idx_params['rotate'] if 'rotate' in idx_params else 0.5
            # The transfer frame into which 3D data (pointclouds) are transformed for the pointcloud data topic
            self.status_msg.frame_3d = idx_params['frame_3d'] if 'frame_3d' in idx_params else "nepi_center_frame"
        
        self.status_pub.publish(self.status_msg)
    
