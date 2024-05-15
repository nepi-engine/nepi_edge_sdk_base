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

from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow
from nepi_ros_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryResponse

from nepi_edge_sdk_base import nepi_img

class ROSIDXSensorIF:
    # Default Global Values
    IDX_CONTROLS_ENABLE = True
    AUTO_ADJUST = False
    RESOLUTION_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    FRAMERATE_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    SETTINGS_STATE_LIST = []
    
    # Default Factory Values for Post Processing
    PP_AUTO_ADJUST = False
    PP_BRIGHTNESS_RATIO = 0.5
    PP_CONTRAST_RATIO =  0.5
    PP_THRESHOLD_RATIO =  0.0
    PP_RESOLUTION_MODE = 3 # LOW, MED, HIGH, MAX
    PP_FRAMERATE_MODE = 3 # LOW, MED, HIGH, MAX

    # Default Factory Values for Callbacks
    CB_AUTO_ADJUST = False
    CB_BRIGHTNESS_RATIO = 0.5
    CB_CONTRAST_RATIO =  0.5
    CB_THRESHOLD_RATIO =  0.5
    CB_RESOLUTION_MODE = 1 # LOW, MED, HIGH, MAX
    CB_FRAMERATE_MODE = 1 # LOW, MED, HIGH, MAX

    init_idx_controls_enable = None
    init_auto_adjust = None
    init_brightness = None
    init_contrast = None
    init_thresholding = None
    init_resolution = None
    init_framerate = None
    init_min_range = None
    init_max_range = None



    def resetControlsCb(self, msg):
        rospy.loginfo("Resetting IDX and Sensor Controls")
        self.resetControls()

    def resetControls(self):
        rospy.set_param('~idx/idx_controls', self.init_idx_controls)
        self.status_msg.idx_controls = self.init_idx_controls

        rospy.set_param('~idx/auto', self.init_auto_adjust)
        self.status_msg.auto = self.init_auto_adjust
        
        rospy.set_param('~idx/brightness', self.init_brightness)
        self.status_msg.brightness = self.init_brightness
        
        rospy.set_param('~idx/contrast', self.init_contrast)
        self.status_msg.contrast = self.init_contrast
        
        rospy.set_param('~idx/thresholding', self.init_thresholding)
        self.status_msg.thresholding = self.init_thresholding
        
        rospy.set_param('~idx/resolution_mode', self.init_resolution)
        self.status_msg.resolution_mode = self.init_resolution
        
        rospy.set_param('~idx/framerate_mode', self.init_framerate)
        self.status_msg.framerate_mode = self.init_framerate

        self.updateAndPublishStatus(do_updates=False) # Updated inline here


    def updateSettings(self,msg):
        new_settings = msg.data
        rospy.set_param('~idx/settings', new_settings)
        self.status_msg.settings_states = new_settings
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setIDXControlsEnable(self, msg):
        new_idx_controls = msg.data
        if new_idx_controls:
            rospy.loginfo("Enabling IDX Controls")
        else:
            rospy.loginfo("Disabling IDX Controls")
            self.resetControls()
        rospy.set_param('~idx/idx_controls', new_idx_controls)
        self.status_msg.idx_controls = new_idx_controls
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setAutoAdjust(self, msg):
        new_auto = msg.data
        print(new_auto)
        status = False
        if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
            if new_auto:
                rospy.loginfo("Enabling IDX Auto Adjust")
            else:
                rospy.loginfo("Disabling IDX Auto Adjust")
            rospy.loginfo(self.setAutoAdjustCb)
            if (self.setAutoAdjustCb is not None):
                rospy.loginfo("Using Auto Adjust driver callback")
                # Call the parent's method and update ROS param as necessary
                # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                status, err_str = self.setAutoAdjustCb(new_auto)
                if status is False:
                    rospy.logerr("Failed to update auto_adjust: " + err_str)
            else:
                # Use post processing
                rospy.loginfo("Using Auto Adjust post processing")
                status = True
            if status is True:
                rospy.loginfo("Udpating local status")
                rospy.set_param('~idx/auto', new_auto)
                self.status_msg.auto = new_auto
                if new_auto:
                    # Reset brightness, contrast, and threshold
                    rospy.set_param('~idx/brightness', self.init_brightness)
                    self.status_msg.brightness = self.init_brightness
                    
                    rospy.set_param('~idx/contrast', self.init_contrast)
                    self.status_msg.contrast = self.init_contrast
                    
                    rospy.set_param('~idx/thresholding', self.init_thresholding)
                    self.status_msg.thresholding = self.init_thresholding
        self.updateAndPublishStatus(do_updates=False) # Updated inline here


    def setBrightness(self, msg):
        new_brightness = msg.data
        status = False
        if (new_brightness < 0.0 or new_brightness > 1.0):
            rospy.logerr("Brightness value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                if rospy.get_param('~idx/auto', self.AUTO_ADJUST):
                    pass # Skip updating in automode
                elif (self.setBrightnessCb is not None):
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setBrightnessCb(new_brightness)
                    if status is False:
                        rospy.logerr("Failed to update brightness: " + err_str)
                else:
                    # Use post processing
                    status = True
                if status is True:
                    rospy.set_param('~idx/brightness', new_brightness)
                    self.status_msg.brightness = new_brightness
            self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setContrast(self, msg):
        new_contrast = msg.data
        status = False
        if (new_contrast < 0.0 and new_contrast != -1.0) or (new_contrast > 1.0):
            rospy.logerr("Contrast value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                if rospy.get_param('~idx/auto', self.AUTO_ADJUST):
                    pass # Skip updating in automode
                elif (self.setContrastCb is not None):
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setContrastCb(new_contrast)
                    if status is False:
                        rospy.logerr("Failed to update contrast: " + err_str)
                else:
                    # Use post processing
                    status = True
                if status is True:
                    rospy.set_param('~idx/contrast', new_contrast)
                    self.status_msg.contrast = new_contrast
            self.updateAndPublishStatus(do_updates=False) # Updated inline here

 

    def setThresholding(self, msg):
        new_thresholding = msg.data
        status = False
        if (new_thresholding < 0.0 or new_thresholding > 1.0):
            rospy.logerr("Thresholding value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                if rospy.get_param('~idx/auto', self.AUTO_ADJUST):
                    pass # Skip updating in automode
                elif (self.setThresholdingCb is not None):
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setThresholdingCb(new_thresholding)
                    if status is False:
                        rospy.logerr("Failed to update thresholding: " + err_str)
                else:
                    # Use post processing
                    status = True
                if status is True:
                    rospy.set_param('~idx/thresholding', new_thresholding)
                    self.status_msg.thresholding = new_thresholding
            self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setResolutionMode(self, msg):
        new_resolution = msg.data
        status = False
        if (new_resolution < 0 or new_resolution > 3):
            rospy.logerr("Resolution mode value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                if (self.setResolutionModeCb is not None):
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setResolutionModeCb(new_resolution)
                    if status is False:
                        rospy.logerr("Failed to update resolution: " + err_str)
                else:
                    # Use post processing
                    status = True
                if status is True:
                    rospy.set_param('~idx/resolution_mode', new_resolution)
                    self.status_msg.resolution_mode = new_resolution
            self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setFramerateMode(self, msg):
        new_framerate = msg.data
        status = False
        if (new_framerate < 0 or new_framerate > 3):
            rospy.logerr("Framerate mode value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                if (self.setResolutionModeCb is not None):
                    # Call the parent's method and update ROS param as necessary
                    # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
                    status, err_str = self.setFramerateModeCb(new_framerate)
                    if status is False:
                        rospy.logerr("Failed to update framerate: " + err_str)
                else:
                    # Use post processing
                    status = True
                if status is True:
                    rospy.set_param('~idx/framerate_mode', new_framerate)
                    self.status_msg.framerate_mode = new_framerate
            self.updateAndPublishStatus(do_updates=False) # Updated inline here

 
    def setRange(self, msg):
        pass # TODO

    def setCurrentSettingsAsDefault(self):
        pass # We only use the param server, no member variables to apply to param server

    def updateFromParamServer(self):
        param_dict = rospy.get_param('~idx', {})
        #rospy.logwarn("Debugging: param_dict = " + str(param_dict))
        if (self.updateSettingsCb is not None and 'settings' in param_dict):
            self.updateSettingsCb(param_dict['settings'])
        if (self.setAutoAdjustCb is not None and 'auto' in param_dict):
            self.setAutoAdjustCb(param_dict['auto'])
        if (self.setBrightnessCb is not None and 'brightness' in param_dict):
            self.setBrightnessCb(param_dict['brightness'])
        if (self.setContrastCb is not None and 'contrast' in param_dict):
            self.setContrastCb(param_dict['contrast'])
        if (self.setThresholdingCb is not None and 'thresholding' in param_dict):
            self.setThresholdingCb(param_dict['thresholding'])
        if (self.setResolutionModeCb is not None and 'resolution_mode' in param_dict):
            self.setResolutionModeCb(param_dict['resolution_mode'])
        if (self.setFramerateModeCb is not None and 'framerate_mode' in param_dict):
            self.setFramerateModeCb(param_dict['framerate_mode'])
        if (self.setRangeCb is not None and 'start_range' in param_dict and 'stop_range' in param_dict):
            self.setRangeCb(param_dict['start_range'], param_dict['stop_range'])

        self.updateAndPublishStatus()

    def provide_capabilities(self, _):
        return self.capabilities_report
           
    def __init__(self, sensor_name, 
                 setAutoAdjustCb=None,
                 setContrastCb=None, setBrightnessCb=None, setThresholdingCb=None,
                 setResolutionModeCb=None, setFramerateModeCb=None, setRangeCb=None, 
                 getColor2DImgCb=None, stopColor2DImgAcquisitionCb=None, 
                 getGrayscale2DImgCb=None, stopGrayscale2DImgAcquisitionCb=None,
                 getDepthMapCb=None, stopDepthMapAcquisitionCb=None, 
                 getDepthImgCb=None, stopDepthImgAcquisitionCb=None,
                 getPointcloudCb=None, stopPointcloudAcquisitionCb=None, 
                 getPointcloudImgCb=None, stopPointcloudImgAcquisitionCb=None):
        
        data_products = []
        self.sensor_name = sensor_name
        self.camera_frame_id = sensor_name + "_frame" # TODO: Configurable?

        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?
        self.cv_bridge = CvBridge()

        self.capabilities_report = IDXCapabilitiesQueryResponse()

        #************************************ new settings testing
        
        self.settings_options = ["TestDiscrete","Discrete","Option_1","Option_2","Option_3","Option_4",
  			"TestString","String",
  			"TestBool","Bool",
  			"TestInt","Int","0","100",
  			"TestFloat","Float"]

        self.updateSettingsCb = None
        


        self.capabilities_report.settings_options = str(self.settings_options)
       

        self.settings_state_list = ["TestDiscrete","Discrete","Option_1",
  			"TestString","String","InitString",
  			"TestBool","Bool","True",
  			"TestInt","Int","5",
  			"TestFloat","Float","3.14"]   

        rospy.Subscriber('~idx/update_settings', String, self.updateSettings, queue_size=1)
        self.init_settings_state = rospy.get_param('~idx/settings', self.settings_state_list)
        rospy.set_param('~idx/settings', self.init_settings_state )
        #************************************ new settings testing

        # Set up standard IDX parameters with ROS param and subscriptions
        # Defer actually setting these on the camera via the parent callbacks... the parent may need to do some 
        # additional setup/calculation first. Parent can then get these all applied by calling updateFromParamServer()

        
        

        rospy.Subscriber('~idx/reset_controls', Empty, self.resetControlsCb, queue_size=1)

        rospy.Subscriber('~idx/set_idx_controls_enable', Bool, self.setIDXControlsEnable, queue_size=1)
        self.init_idx_controls = rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE)
        rospy.set_param('~idx/idx_controls', self.init_idx_controls)

        self.setAutoAdjustCb = setAutoAdjustCb
        rospy.Subscriber('~idx/set_auto_adjust', Bool, self.setAutoAdjust, queue_size=1)
        if (self.setAutoAdjustCb is not None):        
            self.init_auto_adjust = rospy.get_param('~idx/auto', self.CB_AUTO_ADJUST)
        else:
            self.init_auto_adjust = rospy.get_param('~idx/auto', self.PP_AUTO_ADJUST)
        rospy.set_param('~idx/auto', self.init_auto_adjust)
        self.capabilities_report.has_auto_adjustment = True
        

        self.setBrightnessCb = setBrightnessCb
        rospy.Subscriber('~idx/set_brightness', Float32, self.setBrightness, queue_size=1)
        if (self.setBrightnessCb is not None):        
            self.init_brightness = rospy.get_param('~idx/brightness', self.CB_BRIGHTNESS_RATIO)
        else:
            self.init_brightness = rospy.get_param('~idx/brightness', self.PP_BRIGHTNESS_RATIO)
        rospy.set_param('~idx/brightness', self.init_brightness)
        self.capabilities_report.adjustable_brightness = True


        self.setContrastCb = setContrastCb
        rospy.Subscriber('~idx/set_contrast', Float32, self.setContrast, queue_size=1)
        if (self.setContrastCb is not None):        
            self.init_contrast = rospy.get_param('~idx/contrast', self.CB_CONTRAST_RATIO)
        else:
            self.init_contrast = rospy.get_param('~idx/contrast', self.PP_CONTRAST_RATIO)
        rospy.set_param('~idx/contrast', self.init_contrast)
        self.capabilities_report.adjustable_contrast = True
        

        self.setThresholdingCb = setThresholdingCb
        rospy.Subscriber('~idx/set_thresholding', Float32, self.setThresholding, queue_size=1)
        if (self.setThresholdingCb is not None):        
            self.init_thresholding = rospy.get_param('~idx/thresholding', self.CB_THRESHOLD_RATIO)
        else:
            self.init_thresholding = rospy.get_param('~idx/thresholding', self.PP_THRESHOLD_RATIO)
        rospy.set_param('~idx/thresholding', self.init_thresholding)
        self.capabilities_report.adjustable_thresholding = True

        self.setResolutionModeCb = setResolutionModeCb
        rospy.Subscriber('~idx/set_resolution_mode', UInt8, self.setResolutionMode, queue_size=1)
        if (self.setResolutionModeCb is not None):
            self.init_resolution = rospy.get_param('~idx/resolution_mode', self.CB_RESOLUTION_MODE)
        else:
            self.init_resolution = rospy.get_param('~idx/resolution_mode', self.PP_RESOLUTION_MODE)
        rospy.set_param('~idx/resolution_mode', self.init_resolution)
        self.capabilities_report.adjustable_resolution = True
               
        self.setFramerateModeCb = setFramerateModeCb
        rospy.Subscriber('~idx/set_framerate_mode', UInt8, self.setFramerateMode, queue_size=1)
        if (self.setFramerateModeCb is not None):
            self.init_framerate = rospy.get_param('~idx/framerate_mode', self.CB_FRAMERATE_MODE) 
        else:
            self.init_framerate = rospy.get_param('~idx/framerate_mode', self.PP_FRAMERATE_MODE)
        rospy.set_param('~idx/framerate_mode', self.init_framerate)
        self.capabilities_report.adjustable_framerate = True


        self.setRangeCb = setRangeCb
        if (self.setRangeCb is not None):        
            rospy.Subscriber('~idx/set_range_window', RangeWindow, self.setRange, queue_size=1)
            self.init_start_range = rospy.get_param('~idx/range_window/start_range', 0.0)
            rospy.set_param('~idx/range_window/start_range', self.init_start_range)
            self.init_stop_range = rospy.get_param('~idx/range_window/stop_range', 1.0)
            rospy.set_param('~idx/range_window/stop_range', self.init_stop_range)
            self.capabilities_report.adjustable_range = True
        else:
            self.capabilities_report.adjustable_range = False
        
        # Start the data producer threads
        self.getColor2DImgCb = getColor2DImgCb
        if (self.getColor2DImgCb is not None):
            self.color_img_pub = rospy.Publisher('~idx/color_2d_image', Image, queue_size=1, tcp_nodelay=True)
            data_products.append('color_2d_image')
            self.color_img_thread = threading.Thread(target=self.runColorImgThread)
            self.color_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.capabilities_report.has_color_2d_image = True
        else:
            self.capabilities_report.has_color_2d_image = False
        self.stopColor2DImgAcquisitionCb = stopColor2DImgAcquisitionCb

        self.getGrayscale2DImgCb = getGrayscale2DImgCb
        if (self.getGrayscale2DImgCb is not None):
            self.grayscale_img_pub = rospy.Publisher('~idx/bw_2d_image', Image, queue_size = 3)
            data_products.append('bw_2d_image')
            self.bw_img_thread = threading.Thread(target=self.runGrayscaleImgThread)
            self.bw_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.capabilities_report.has_bw_2d_image = True
        else:
            self.capabilities_report.has_bw_2d_image = False
        self.stopGrayscale2DImgCb = stopGrayscale2DImgAcquisitionCb

        self.getDepthMapCb = getDepthMapCb
        if (self.getDepthMapCb is not None):
            self.depth_map_pub = rospy.Publisher('~idx/depth_map', Image, queue_size=1)
            data_products.append('depth_map')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to create depth map thread")
            self.capabilities_report.has_depth_map = True
        else:
            self.capabilities_report.has_depth_map = False
        self.stopDepthMapAcquisitionCb = stopDepthMapAcquisitionCb

        self.getDepthImgCb = getDepthImgCb
        if (self.getDepthImgCb is not None):
            self.depth_img_pub = rospy.Publisher('~idx/depth_image', Image, queue_size = 3)
            data_products.append('depth_image')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to create depth image thread")
            self.capabilities_report.has_depth_image = True
        else:
            self.capabilities_report.has_depth_image = False
        self.stopDepthImgAcquisitionCb = stopDepthImgAcquisitionCb

        self.getPointcloudImgCb = getPointcloudImgCb
        if (self.getPointcloudImgCb is not None):
            self.pointcloud_img_pub = rospy.Publisher('~idx/pointcloud_image', Image, queue_size=1)
            data_products.append('pointcloud_image')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to create pointcloud image thread")
            self.capabilities_report.has_pointcloud_image = True
        else:
            self.capabilities_report.has_pointcloud_image = False
        self.stopPointcloudAcquisitionCb = stopPointcloudAcquisitionCb        

        self.getPointcloudCb = getPointcloudCb
        if (self.getPointcloudCb is not None):
            self.pointcloud_img_pub = rospy.Publisher('~idx/pointcloud', PointCloud2, queue_size=1)
            data_products.append('pointcloud')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to create pointcloud thread")
            self.capabilities_report.has_pointcloud = True
        else:
            self.capabilities_report.has_pointcloud = False
        self.stopPointcloudImgAcquisitionCb = stopPointcloudImgAcquisitionCb   

        # Set up additional publishers
        self.status_msg = IDXStatus()
        self.status_pub = rospy.Publisher('~idx/status', IDXStatus, queue_size=1, latch=True)

        # Set up the save data and save cfg i/f -- Do this before launching threads so that they can check data_product_should_save() immediately
        self.save_data_if = SaveDataIF(data_product_names = data_products)
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer)

        # Launch the acquisition threads
        self.color_img_thread.start()
        self.bw_img_thread.start()  
        # TODO: Start other acquisition threads here when they are implemented  

        # Set up service providers
        rospy.Service('~idx/capabilities_query', IDXCapabilitiesQuery, self.provide_capabilities)

    def runColorImgThread(self):
        rospy.loginfo(rospy.get_name() + ": starting color_2d_image acquisition thread")
        acquiring = False
        while (True):
            saving_is_enabled = self.save_data_if.data_product_saving_enabled('color_2d_image')
            has_subscribers = (self.color_img_pub.get_num_connections() > 0)
            if (has_subscribers is True) or (saving_is_enabled is True):
                acquiring = True
                status, msg, cv2_img, ros_timestamp = self.getColor2DImgCb()
                if (status is False):
                    rospy.logerr_throttle(1, msg)
                    continue
   
                if (has_subscribers is True):
                    # Convert cv to ros and publish
                    start = time.time()
                    if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                        # Apply IDX Abstracted Controls
                        if self.setResolutionModeCb is None:
                            resolution_mode = rospy.get_param('~idx/resolution_mode', self.PP_RESOLUTION_MODE) 
                            resolution_ratio = resolution_mode/3
                            [cv2_img,new_res] = nepi_img.adjust_resolution(cv2_img, resolution_ratio)
                        if rospy.get_param('~idx/auto', self.AUTO_ADJUST) is False:
                            if self.setBrightnessCb is None:
                                brightness_ratio = rospy.get_param('~idx/brightness', self.PP_BRIGHTNESS_RATIO)
                                cv2_img = nepi_img.adjust_brightness(cv2_img, brightness_ratio)
                            if self.setContrastCb is None:
                                contrast_ratio = rospy.get_param('~idx/contrast', self.PP_CONTRAST_RATIO)
                                cv2_img = nepi_img.adjust_contrast(cv2_img, contrast_ratio)
                            if self.setThresholdingCb is None:
                                threshold_ratio = rospy.get_param('~idx/thresholding', self.PP_THRESHOLD_RATIO)
                                cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold_ratio)
                        else:
                            if self.setAutoAdjustCb is None:
                                cv2_img = nepi_img.adjust_auto(cv2_img, 0.3)
                            
                        if self.setFramerateModeCb is None:
                            framerate_mode = rospy.get_param('~idx/framerate_mode', self.PP_FRAMERATE_MODE) 
    ##  Need to get current framerate setting
                            current_fps = 20
    ##  Hard Coded for now
                            framerate_ratio = framerate_mode/3
                            [cv2_img,new_rate] = nepi_img.adjust_framerate(cv2_img, current_fps, framerate_ratio)
                  
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
                if self.stopColor2DImgAcquisitionCb is not None:
                    rospy.loginfo(rospy.get_name() + ": stopping color_2d_image acquisition")
                    self.stopColor2DImgAcquisitionCb()
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
                status, msg, cv2_img, ros_timestamp = self.getGrayscale2DImgCb()
                if (status is False):
                    rospy.logerr_throttle(1, msg)
                    continue

                acquiring = True
                
                if (has_subscribers is True):

                    start = time.time()
                    if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                        # Apply IDX Abstracted Controls
                        if self.setResolutionModeCb is None:
                            resolution_mode = rospy.get_param('~idx/resolution_mode', self.PP_RESOLUTION_MODE) 
                            resolution_ratio = resolution_mode/3
                            [cv2_img,new_res] = nepi_img.adjust_resolution(cv2_img, resolution_ratio)
                        if rospy.get_param('~idx/auto', self.AUTO_ADJUST) is False:
                            if self.setBrightnessCb is None:
                                brightness_ratio = rospy.get_param('~idx/brightness', self.PP_BRIGHTNESS_RATIO)
                                cv2_img = nepi_img.adjust_brightness(cv2_img, brightness_ratio)
                            if self.setContrastCb is None:
                                contrast_ratio = rospy.get_param('~idx/contrast', self.PP_CONTRAST_RATIO)
                                cv2_img = nepi_img.adjust_contrast(cv2_img, contrast_ratio)
                            if self.setThresholdingCb is None:
                                threshold_ratio = rospy.get_param('~idx/thresholding', self.PP_THRESHOLD_RATIO)
                                cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold_ratio)
                        else:
                            if self.setAutoAdjustCb is None:
                                cv2_img = nepi_img.adjust_auto(cv2_img, 0.5)
                            
                        if self.setFramerateModeCb is None:
                            framerate_mode = rospy.get_param('~idx/framerate_mode', self.PP_FRAMERATE_MODE) 
    ##  Need to get current framerate setting
                            current_fps = 20
    ##  Hard Coded for now
                            framerate_ratio = framerate_mode/3
                            [cv2_img,new_rate] = nepi_img.adjust_framerate(cv2_img, current_fps, framerate_ratio)


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
                if self.stopGrayscale2DImgCb is not None:
                    rospy.loginfo(rospy.get_name() + ": stopping bw_2d_image acquisition")
                    self.stopGrayscale2DImgCb()
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
            self.status_msg.settings_states = str(idx_params['settings']) if 'settings' in idx_params else ""
            self.status_msg.idx_controls = idx_params['idx_controls'] if 'idx_controls' in idx_params else 0
            self.status_msg.auto = idx_params['auto'] if 'auto' in idx_params else 0
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
    
