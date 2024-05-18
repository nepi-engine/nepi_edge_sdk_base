#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import time
import threading
import rospy
from cv_bridge import CvBridge
import cv2

from std_msgs.msg import UInt8, Float32, Bool, Empty, String
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow
from nepi_ros_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryResponse, NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse

from nepi_edge_sdk_base import nepi_img
from nepi_edge_sdk_base import nepi_nex

class ROSIDXSensorIF:
    # Default Global Values

    RESOLUTION_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    FRAMERATE_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    SETTINGS_STATE_LIST = []
    UPDATE_NAVPOSE_RATE_HZ = 20
    
    # Backup Factory Control Values 
    FACTORY_CONTROLS = dict( controls_enable = True,
        auto_adjust = False,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.5,
        resolution_mode = 1, # LOW, MED, HIGH, MAX
        framerate_mode = 1, # LOW, MED, HIGH, MAX
        start_range_ratio = 0.0,
        stop_ranage_ratio = 1.0,
        min_range_m = 0.0,
        max_range_m = 1.0 
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

    caps_settings = None
    factory_settings = None
    current_settings = None
    init_settings = None
    update_settings_function = None



    def resetSensorCb(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Resetting IDX and Sensor Controls")
        self.resetSensor()

    def resetSensor(self):
        #************************************ new settings testing
        self.updateSettings(self.init_settings)
        #************************************ new settings testing

        rospy.set_param('~idx/controls_enable', self.init_controls_enable)
        self.status_msg.controls_enable = self.init_controls_enable

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

        self.updateAndPublishStatus(do_updates=False) # Updated inline here

#************************************ new settings testing
    def updateSettingsCb(self,msg):
        rospy.loginfo("Received settings update msg " + msg)
        new_settings = nepi_nex.parse_settings_msg_data(msg.data)
        self.updateSettings(new_settings)


    def updateSettings(self,new_settings):
        if self.update_settings_function != None:
            for setting in new_settings:
                [name_match,type_match,value_match] = nepi_nex.compare_setting_in_settings(setting,self.current_settings)
                if not value_match: # name_match would be true for value_match to be true
                    rospy.loginfo("Will try to update setting " + str(setting))
                    [self.current_settings,success] = nepi_nex.try_to_update_setting(setting,self.current_settings,self.cap_settings,self.update_settings_function)
            rospy.set_param('~idx/settings', self.current_settings)
            self.current_settings = nepi_nex.sort_settings_alphabetically(self.current_settings,1)
            settings_msg = nepi_nex.create_msg_data_from_settings(self.current_settings)
            self.status_msg.settings_states = settings_msg 
            self.updateAndPublishStatus(do_updates=False) # Updated inline here
        else:
            rospy.loginfo("Settings updates ignored. No settings update function defined ")
#************************************ new settings testing

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

                    self.resetSensor()
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
        new_start_range = msg.range_window.start_range
        new_stop_range = msg.range_window.stop_range
        if rospy.get_param('~idx/controls_enable', self.init_controls_enable) is False:
            rospy.loginfo("Ignoring Set Range request. Controls disabled")    
        else: 
            if self.setRange is None:
                rospy.loginfo("Ignoring Set Range. Driver has no setRange function")
            else:
                if (new_start_range >= 0 and new_stop_range <= 1 and new_stop_range < new_start_range):
                    rospy.logerr("Range values out of bounds")
                    self.updateAndPublishStatus(do_updates=False) # No change
                    return
                else:
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setRange(new_min_range,new_max_range)
                    if status is True:
                        rospy.set_param('~idx/range_window/startRange', new_start_range)
                        rospy.set_param('~idx/range_window/startRange', new_stop_range)
                        self.status_msg.range_window.start_range = new_start_range
                        self.status_msg.range_window.stop_range = new_stop_range
                    else:
                        rospy.logerr("Failed to update framerate: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

        
    def pubNavposeCb(self,timer):
        if self.getGPSMsg != None:
            self.idx_navpose_gps_pub.publish(self.getGPSMsg())
    
        if self.getOdomMsg != None:
            self.idx_navpose_odom_pub.publish(self.getOdomMsg())

        if self.getHeadingMsg != None:
            self.idx_navpose_gps_pub.publish(self.getHeadingMsg())            
   

    def setCurrentSettingsAsDefault(self):
        pass # We only use the param server, no member variables to apply to param server

    def updateFromParamServer(self):
        param_dict = rospy.get_param('~idx', {})
        #rospy.logwarn("Debugging: param_dict = " + str(param_dict))
        if (self.updateSettings is not None and 'settings' in param_dict):
            self.updateSettings(param_dict['settings'])
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
            self.setRange(param_dict['start_range'], param_dict['stop_range'])

        self.updateAndPublishStatus()

    def provide_capabilities(self, _):
        return self.capabilities_report
    
    def provide_navpose_capabilities(self, _):
        return self.navpose_capabilities_report
           
    def __init__(self, sensor_name, factoryControls = None, capSettings=None, 
                 factorySettings=None, currentSettings=None, settingsUpdateFunction=None, 
                 setControlsEnable=None, setAutoAdjust=None,
                 setContrast=None, setBrightness=None, setThresholding=None,
                 setResolutionMode=None, setFramerateMode=None, setRange=None, 
                 getColor2DImg=None, stopColor2DImgAcquisition=None, 
                 getGrayscale2DImg=None, stopGrayscale2DImgAcquisition=None,
                 getDepthMap=None, stopDepthMapAcquisition=None, 
                 getDepthImg=None, stopDepthImgAcquisition=None,
                 getPointcloud=None, stopPointcloudAcquisition=None, 
                 getPointcloudImg=None, stopPointcloudImgAcquisition=None,
                 getGPSMsg=None,getOdomMsg=None,getHeadingMsg=None):
        
        data_products = []
        self.sensor_name = sensor_name
        self.camera_frame_id = sensor_name + "_frame" # TODO: Configurable?

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




        # Set up standard IDX parameters with ROS param and subscriptions
        # Defer actually setting these on the camera via the parent callbacks... the parent may need to do some 
        # additional setup/calculation first. Parent can then get these all applied by calling updateFromParamServer()

        rospy.Subscriber('~idx/reset_sensor', Empty, self.resetSensorCb, queue_size=1) # start local callback

        #************************************ new settings testing
        # Initialize Sensor Settings from Node
        if capSettings is None:
            self.cap_settings = nepi_nex.NONE_SETTINGS
        else:
            self.cap_settings = capSettings

        if currentSettings is None:
            self.current_settings = nepi_nex.NONE_SETTINGS
        else:
            self.current_settings = current_settings

        if factorySettings is None:
            self.factory_settings = nepi_nex.NONE_SETTINGS
        else:
            self.factory_settings = factorySettings

        if settingsUpdateFunction is None:
            self.update_settings_function = None
        else:
            self.update_settings_function = settingsUpdateFunction


        ####  Remove after testing
        self.cap_settings = nepi_nex.TEST_CAP_SETTINGS ####  Remove after testing
        self.current_settings = nepi_nex.TEST_SETTINGS ####  Remove after testing
        self.factory_settings = nepi_nex.TEST_SETTINGS ####  Remove after testing
        self.update_settings_function = nepi_nex.TEST_UPDATE_FUNCTION_SUCCESS ####  Remove after testing
        #self.update_settings_function = nepi_nex.TEST_UPDATE_FUNCTION_FAIL ####  Remove after testing
        #self.update_settings_function = nepi_nex.TEST_UPDATE_FUNCTION_EXEPTION ####  Remove after testing
        ####  Remove after testing

        self.cap_settings = nepi_nex.sort_settings_alphabetically(self.cap_settings,1)
        self.current_settings= nepi_nex.sort_settings_alphabetically(self.current_settings,1)
        self.factory_settings= nepi_nex.sort_settings_alphabetically(self.factory_settings,1)

        cap_msg = nepi_nex.create_msg_data_from_settings(self.cap_settings)
        self.capabilities_report.settings_options = cap_msg

        self.init_settings = rospy.get_param('~idx/settings', self.factory_settings)
        rospy.set_param('~idx/settings', self.init_settings )

        rospy.Subscriber('~idx/update_settings', String, self.updateSettingsCb, queue_size=1) # start local callbac

        
        #************************************ new settings testing        
        self.setControlsEnable = setControlsEnable
        if setControlsEnable is not None:
            rospy.Subscriber('~idx/set_controls_enable', Bool, self.setControlsEnableCb, queue_size=1) # start local callback
        self.init_controls_enable = rospy.get_param('~idx/controls_enable',  self.factory_controls["controls_enable"])
        rospy.set_param('~idx/controls_enable', self.init_controls_enable)
        

        self.setAutoAdjust = setAutoAdjust
        self.init_auto_adjust = rospy.get_param('~idx/auto',  self.factory_controls["auto_adjust"])
        rospy.set_param('~idx/auto', self.init_auto_adjust)
        if setAutoAdjust is not None:
            rospy.Subscriber('~idx/set_auto_adjust', Bool, self.setAutoAdjustCb, queue_size=1) # start local callback
            self.capabilities_report.has_auto_adjustment = True
        else:
            self.capabilities_report.has_auto_adjustment = False
       
        self.setBrightness = setBrightness
        self.init_brightness_ratio = rospy.get_param('~idx/brightness',  self.factory_controls["brightness_ratio"])
        rospy.set_param('~idx/brightness', self.init_brightness_ratio)
        if setBrightness is not None:
            rospy.Subscriber('~idx/set_brightness', Float32, self.setBrightnessCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_brightness = True
        else:
            self.capabilities_report.adjustable_brightness = False

        self.setContrast = setContrast
        self.init_contrast_ratio = rospy.get_param('~idx/contrast',  self.factory_controls["contrast_ratio"])
        rospy.set_param('~idx/contrast', self.init_contrast_ratio)
        if setContrast is not None:
            rospy.Subscriber('~idx/set_contrast', Float32, self.setContrastCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_contrast = True
        else:
            self.capabilities_report.adjustable_contrast = False
        

        self.setThresholding = setThresholding
        self.init_threshold_ratio = rospy.get_param('~idx/thresholding',  self.factory_controls["threshold_ratio"])
        rospy.set_param('~idx/thresholding', self.init_threshold_ratio)        
        if setThresholding is not None:
            rospy.Subscriber('~idx/set_thresholding', Float32, self.setThresholdingCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_thresholding = True
        else:
            self.capabilities_report.adjustable_thresholding = False

        self.setResolutionMode = setResolutionMode
        self.init_resolution_mode = rospy.get_param('~idx/resolution_mode',  self.factory_controls["resolution_mode"])
        rospy.set_param('~idx/resolution_mode', self.init_resolution_mode)
        if setResolutionMode is not None:
            rospy.Subscriber('~idx/set_resolution_mode', UInt8, self.setResolutionModeCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_resolution = True
        else:
            self.capabilities_report.adjustable_resolution = False
               
        self.setFramerateMode = setFramerateMode
        self.init_framerate_mode = rospy.get_param('~idx/framerate_mode',  self.factory_controls["framerate_mode"])
        rospy.set_param('~idx/framerate_mode', self.init_framerate_mode)
        if setFramerateMode is not None:
            rospy.Subscriber('~idx/set_framerate_mode', UInt8, self.setFramerateModeCb, queue_size=1) # start local callback
            self.capabilities_report.adjustable_framerate = True
        else:
            self.capabilities_report.adjustable_framerate = False


        self.setRange = setRange
        self.init_start_range = rospy.get_param('~idx/range_window/start_range',  self.factory_controls[control])
        rospy.set_param('~idx/range_window/start_range', self.init_start_range)
        self.init_stop_range = rospy.get_param('~idx/range_window/stop_range', 1.0)
        rospy.set_param('~idx/range_window/stop_range', self.init_stop_range)
        if setRange is not None:        
            rospy.Subscriber('~idx/set_range_window', RangeWindow, self.setRangeCb, queue_size=1)
            self.capabilities_report.adjustable_range = True
        else:
            self.capabilities_report.adjustable_range = False
        
        # Start the data producer threads
        self.getColor2DImg = getColor2DImg
        if (self.getColor2DImg is not None):
            self.color_img_pub = rospy.Publisher('~idx/color_2d_image', Image, queue_size=1, tcp_nodelay=True)
            data_products.append('color_2d_image')
            self.color_img_thread = threading.Thread(target=self.runColorImgThread)
            self.color_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.capabilities_report.has_color_2d_image = True
        else:
            self.capabilities_report.has_color_2d_image = False
        self.stopColor2DImgAcquisition = stopColor2DImgAcquisition

        self.getGrayscale2DImg = getGrayscale2DImg
        if (self.getGrayscale2DImg is not None):
            self.grayscale_img_pub = rospy.Publisher('~idx/bw_2d_image', Image, queue_size = 3)
            data_products.append('bw_2d_image')
            self.bw_img_thread = threading.Thread(target=self.runGrayscaleImgThread)
            self.bw_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.capabilities_report.has_bw_2d_image = True
        else:
            self.capabilities_report.has_bw_2d_image = False
        self.stopGrayscale2DImg = stopGrayscale2DImgAcquisition

        self.getDepthMap = getDepthMap
        if (self.getDepthMap is not None):
            self.depth_map_pub = rospy.Publisher('~idx/depth_map', Image, queue_size=1)
            data_products.append('depth_map')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to create depth map thread")
            self.capabilities_report.has_depth_map = True
        else:
            self.capabilities_report.has_depth_map = False
        self.stopDepthMapAcquisition = stopDepthMapAcquisition

        self.getDepthImg = getDepthImg
        if (self.getDepthImg is not None):
            self.depth_img_pub = rospy.Publisher('~idx/depth_image', Image, queue_size = 3)
            data_products.append('depth_image')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to create depth image thread")
            self.capabilities_report.has_depth_image = True
        else:
            self.capabilities_report.has_depth_image = False
        self.stopDepthImgAcquisition = stopDepthImgAcquisition

        self.getPointcloudImg = getPointcloudImg
        if (self.getPointcloudImg is not None):
            self.pointcloud_img_pub = rospy.Publisher('~idx/pointcloud_image', Image, queue_size=1)
            data_products.append('pointcloud_image')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to create pointcloud image thread")
            self.capabilities_report.has_pointcloud_image = True
        else:
            self.capabilities_report.has_pointcloud_image = False
        self.stopPointcloudAcquisition = stopPointcloudAcquisition        

        self.getPointcloud = getPointcloud
        if (self.getPointcloud is not None):
            self.pointcloud_img_pub = rospy.Publisher('~idx/pointcloud', PointCloud2, queue_size=1)
            data_products.append('pointcloud')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to create pointcloud thread")
            self.capabilities_report.has_pointcloud = True
        else:
            self.capabilities_report.has_pointcloud = False
        self.stopPointcloudImgAcquisition = stopPointcloudImgAcquisition   


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
    
        # Set up additional publishers
        self.status_msg = IDXStatus()
        self.status_pub = rospy.Publisher('~idx/status', IDXStatus, queue_size=1, latch=True)

        if getGPSMsg != None or getOdomMsg != None or getHeadingMsg != None:
            update_navpose_interval_sec = float(1)/self.UPDATE_NAVPOSE_RATE_HZ
            rospy.Timer(rospy.Duration(update_navpose_interval_sec), self.pubNavposeCB)

        # Set up the save data and save cfg i/f -- Do this before launching threads so that they can check data_product_should_save() immediately
        self.save_data_if = SaveDataIF(data_product_names = data_products)
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer)

        # Launch the acquisition threads
        self.color_img_thread.start()
        self.bw_img_thread.start()  
        # TODO: Start other acquisition threads here when they are implemented  

        # Set up service providers
        rospy.Service('~idx/capabilities_query', IDXCapabilitiesQuery, self.provide_capabilities)
        rospy.Service('~idx/navpose_capabilities_query', NavPoseCapabilitiesQuery, self.provide_navpose_capabilities)

    def runColorImgThread(self):
        rospy.loginfo(rospy.get_name() + ": starting color_2d_image acquisition thread")
        acquiring = False
        while (True):
            saving_is_enabled = self.save_data_if.data_product_saving_enabled('color_2d_image')
            has_subscribers = (self.color_img_pub.get_num_connections() > 0)
            if (has_subscribers is True) or (saving_is_enabled is True):
                acquiring = True
                status, msg, cv2_img, ros_timestamp = self.getColor2DImg()
                if (status is False):
                    rospy.logerr_throttle(1, msg)
                    continue
   
                if (has_subscribers is True):
                    # Convert cv to ros and publish                
                    ros_img = self.cv_bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")
                    ros_img.header.stamp = ros_timestamp
                    ros_img.header.frame_id = self.camera_frame_id
                    stop = time.time()

                   
                    #print('CV2Img: ', stop-start)
                    self.color_img_pub.publish(ros_img)
                    next_stop = time.time()
                    #print('Pub: ', next_stop-stop)

                if (self.save_data_if.data_product_should_save('color_2d_image') is True):
                    full_path_filename = self.save_data_if.get_full_path_filename(self.save_data_if.get_timestamp_string(), 
                                                                                  self.sensor_name + '_color_2d_img', 'png')
                    #rospy.logwarn("Debugging: Time to save color_2d_img to " + full_path_filename)
                    cv2.imwrite(full_path_filename, cv2_img)

            elif acquiring is True:
                if self.stopColor2DImgAcquisition is not None:
                    rospy.loginfo(rospy.get_name() + ": stopping color_2d_image acquisition")
                    self.stopColor2DImgAcquisition()
                acquiring = False
            else: # No subscribers and already stopped
                acquiring = False
                rospy.sleep(0.25)

            rospy.sleep(0.01) # Yield

    def runGrayscaleImgThread(self):
        rospy.loginfo(rospy.get_name() + ": starting bw_2d_image acquisition thread")
        acquiring = False
        while (True):
            saving_is_enabled = self.save_data_if.data_product_saving_enabled('bw_2d_image')
            has_subscribers = (self.grayscale_img_pub.get_num_connections() > 0)
            if (has_subscribers is True) or (saving_is_enabled is True):
                status, msg, cv2_img, ros_timestamp = self.getGrayscale2DImg()
                if (status is False):
                    rospy.logerr_throttle(1, msg)
                    continue

                acquiring = True
                
                if (has_subscribers is True):
                    # Convert cv to ros and publish
                    ros_img = self.cv_bridge.cv2_to_imgmsg(cv2_img, encoding="mono8")
                    ros_img.header.stamp = ros_timestamp
                    ros_img.header.frame_id = self.camera_frame_id
                    self.grayscale_img_pub.publish(ros_img)

                if (self.save_data_if.data_product_should_save('bw_2d_image') is True):
                    full_path_filename = self.save_data_if.get_full_path_filename(self.save_data_if.get_timestamp_string(), 
                                                                                  self.sensor_name + '_bw_2d_img', 'png')
                    cv2.imwrite(full_path_filename, cv2_img)

            elif acquiring is True:
                if self.stopGrayscale2DImg is not None:
                    rospy.loginfo(rospy.get_name() + ": stopping bw_2d_image acquisition")
                    self.stopGrayscale2DImg()
                acquiring = False
            else: # No subscribers and already stopped
                acquiring = False
                rospy.sleep(0.25)
                
            rospy.sleep(0.01) # Yield

    def updateAndPublishStatus(self, do_updates = True):
        if do_updates is True:
            # TODO: Probably these should be queried from the parent (and through the driver) via explicit callbacks rather than via the param server
            idx_params = rospy.get_param('~idx')
            rospy.loginfo("IDX settings state string")
            rospy.loginfo(str(idx_params['settings']))
            settings_msg = nepi_nex.create_msg_data_from_settings(idx_params['settings']) if 'settings' in idx_params else 0
            self.status_msg.settings_states = settings_msg 
            self.status_msg.controls_enable = idx_params['controls_enable'] if 'controls_enable' in idx_params else 0
            self.status_msg.auto_adjust = idx_params['auto'] if 'auto' in idx_params else 0
            self.status_msg.resolution_mode = idx_params['resolution_mode'] if 'resolution_mode' in idx_params else 0
            self.status_msg.framerate_mode = idx_params['framerate_mode'] if 'framerate_mode' in idx_params else 0
            self.status_msg.contrast = idx_params['contrast'] if 'contrast' in idx_params else 0
            self.status_msg.brightness = idx_params['brightness'] if 'brightness' in idx_params else 0
            self.status_msg.thresholding = idx_params['thresholding'] if 'thresholding' in idx_params else 0
            self.status_msg.range_window.start_range = idx_params['range_window']['start_range'] if 'range_window' in idx_params else 0
            self.status_msg.range_window.stop_range = idx_params['range_window']['stop_range'] if 'range_window' in idx_params else 0
            self.status_msg.min_range_m = idx_params['range_limits']['min_range_m'] if 'range_limits' in idx_params else 0
            self.status_msg.max_range_m = idx_params['range_limits']['max_range_m'] if 'range_limits' in idx_params else 0
        
            # The transfer frame into which 3D data (pointclouds) are transformed for the pointcloud data topic
            self.status_msg.frame_3d = idx_params['frame_3d'] if 'frame_3d' in idx_params else "nepi_center_frame"
        
        self.status_pub.publish(self.status_msg)
    
