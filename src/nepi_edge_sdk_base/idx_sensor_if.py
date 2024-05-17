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
from nepi_edge_sdk_base import nepi_nex

class ROSIDXSensorIF:
    # Default Global Values

    RESOLUTION_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    FRAMERATE_MODE_MAX = 3 # LOW, MED, HIGH, MAX
    SETTINGS_STATE_LIST = []
    
    # Default Factory Values for Post Processing
    IDX_CONTROLS_ENABLE = True
    AUTO_ADJUST = False
    BRIGHTNESS_RATIO = 0.5
    CONTRAST_RATIO =  0.5
    THRESHOLD_RATIO =  0.0
    RESOLUTION_MODE = 3 # LOW, MED, HIGH, MAX
    FRAMERATE_MODE = 3 # LOW, MED, HIGH, MAX
    MIN_RANGE = 0.0
    MAX_RANGE = 1.0

    # Define class variables
    caps_settings = None
    current_settings = None
    factory_settings = None
    init_settings = None
    update_settings_function = None

    init_idx_controls = None 
    init_auto_adjust = None
    init_brightness = None
    init_contrast = None
    init_thresholding = None
    init_resolution = None
    init_framerate = None
    init_min_range = None
    init_max_range = None



    def resetSensorCb(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Resetting IDX and Sensor Controls")
        self.resetSensor()

    def resetSensor(self):
        #************************************ new settings testing
        self.updateSettings(self.init_settings)
        #************************************ new settings testing

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

#************************************ new settings testing
    def updateSettingsCb(self,msg):
        rospy.loginfo("Received settings update msg " + msg)
        new_settings = nepi_nex.parse_settings_msg_data(msg.data)
        self.updateSettings(new_settings)


    def updateSettings(self,new_settings):
        if self.update_settings_function != None:
            for setting in new_settings:
                [name_match,type_match,value_match] = nepi_nex.check_setting_in_settings(setting,self.current_settings)
                if not value_match: 
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

    def setIDXControlsEnableCb(self, msg):
        rospy.loginfo(msg)
        new_idx_controls = msg.data
        self.setIDXControlsEnable(new_idx_controls)

    def setIDXControlsEnable(self, new_idx_controls):
        if new_idx_controls:
            rospy.loginfo("Enabling IDX Controls")
        else:
            rospy.loginfo("Disabling IDX Controls")
            self.resetSensor()
        rospy.set_param('~idx/idx_controls', new_idx_controls)
        self.status_msg.idx_controls = new_idx_controls
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setAutoAdjustCb(self, msg):
        rospy.loginfo(msg)
        new_auto = msg.data
        self.setAutoAdjust(new_auto)

    def setAutoAdjust(self, new_auto):
        if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
            if new_auto:
                rospy.loginfo("Enabling Auto Adjust")
                # Reset brightness, contrast, and threshold
                rospy.set_param('~idx/brightness', self.init_brightness)
                self.status_msg.brightness = self.init_brightness
                
                rospy.set_param('~idx/contrast', self.init_contrast)
                self.status_msg.contrast = self.init_contrast
                
                rospy.set_param('~idx/thresholding', self.init_thresholding)
                self.status_msg.thresholding = self.init_thresholding
            else:
                rospy.loginfo("Disabling IDX Auto Adjust")
            rospy.set_param('~idx/auto', new_auto)
            self.status_msg.auto = new_auto
        self.updateAndPublishStatus(do_updates=False) # Updated inline here


    def setBrightnessCb(self, msg):
        rospy.loginfo(msg)
        new_brightness = msg.data
        self.setBrightness(new_brightness)

    def setBrightness(self, new_brightness):
        if (new_brightness < 0.0 or new_brightness > 1.0):
            rospy.logerr("Brightness value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                if rospy.get_param('~idx/auto', self.AUTO_ADJUST):
                    pass # Skip updating in automode
                else:
                    rospy.set_param('~idx/brightness', new_brightness)
                    self.status_msg.brightness = new_brightness
            self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setContrastCb(self, msg):
        rospy.loginfo(msg)
        new_contrast = msg.data
        self.setContrast(new_contrast)
 
    def setContrast(self, new_contrast):
        if (new_contrast < 0.0 and new_contrast != -1.0) or (new_contrast > 1.0):
            rospy.logerr("Contrast value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                if rospy.get_param('~idx/auto', self.AUTO_ADJUST):
                    pass # Skip updating in automode
                else:
                    rospy.set_param('~idx/contrast', new_contrast)
                    self.status_msg.contrast = new_contrast
            self.updateAndPublishStatus(do_updates=False) # Updated inline here


    def setThresholdingCb(self, msg):
        rospy.loginfo(msg)
        new_thresholding = msg.data
        self.setThresholding(new_thresholding)

    def setThresholding(self, new_thresholding):
        if (new_thresholding < 0.0 or new_thresholding > 1.0):
            rospy.logerr("Thresholding value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                if rospy.get_param('~idx/auto', self.AUTO_ADJUST):
                    pass # Skip updating in automode
                else:
                    rospy.set_param('~idx/thresholding', new_thresholding)
                    self.status_msg.thresholding = new_thresholding
            self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setResolutionModeCb(self, msg):
        rospy.loginfo(msg)
        new_resolution = msg.data
        self.setResolutionMode(new_resolution)


    def setResolutionMode(self, new_resolution):
        if (new_resolution < 0 or new_resolution > 3):
            rospy.logerr("Resolution mode value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                rospy.set_param('~idx/resolution_mode', new_resolution)
                self.status_msg.resolution_mode = new_resolution
            self.updateAndPublishStatus(do_updates=False) # Updated inline here


        
    def setFramerateModeCb(self, msg):
        rospy.loginfo(msg)
        new_framerate = msg.data
        self.setFramerateMode(new_framerate)

    def setFramerateMode(self, new_framerate):
        if (new_framerate < 0 or new_framerate > 3):
            rospy.logerr("Framerate mode value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        else:
            if rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE):
                rospy.set_param('~idx/framerate_mode', new_framerate)
                self.status_msg.framerate_mode = new_framerate
            self.updateAndPublishStatus(do_updates=False) # Updated inline here

 
    def setRangeCb(self, msg):
        rospy.loginfo(msg)
        start_range = msg.range_window.start_range
        stop_range = msg.range_window.stop_range
        self.setRange

    def setRange(self, start_range, stop_range):
        pass # TODO


    def setCurrentSettingsAsDefault(self):
        pass # We only use the param server, no member variables to apply to param server

    def updateFromParamServer(self):
        param_dict = rospy.get_param('~idx', {})
        #rospy.logwarn("Debugging: param_dict = " + str(param_dict))
        if (self.updateSettingsCb is not None and 'settings' in param_dict):
            self.updateSettings(param_dict['settings'])
        if (self.setAutoAdjustCb is not None and 'auto' in param_dict):
            self.setAutoAdjust(param_dict['auto'])
        if (self.setBrightnessCb is not None and 'brightness' in param_dict):
            self.setBrightness(param_dict['brightness'])
        if (self.setContrastCb is not None and 'contrast' in param_dict):
            self.setContrast(param_dict['contrast'])
        if (self.setThresholdingCb is not None and 'thresholding' in param_dict):
            self.setThresholding(param_dict['thresholding'])
        if (self.setResolutionModeCb is not None and 'resolution_mode' in param_dict):
            self.setResolutionMode(param_dict['resolution_mode'])
        if (self.setFramerateModeCb is not None and 'framerate_mode' in param_dict):
            self.setFramerateMode(param_dict['framerate_mode'])
        if (self.setRangeCb is not None and 'start_range' in param_dict and 'stop_range' in param_dict):
            self.setRange(param_dict['start_range'], param_dict['stop_range'])

        self.updateAndPublishStatus()

    def provide_capabilities(self, _):
        return self.capabilities_report
           
    def __init__(self, sensor_name, capSettings=None, 
                 factorySettings=None, currentSettings=None,
                 settingsUpdateFunction=None, setAutoAdjustCb=None,
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


        rospy.Subscriber('~idx/set_idx_controls_enable', Bool, self.setIDXControlsEnableCb, queue_size=1) # start local callback
        self.init_idx_controls = rospy.get_param('~idx/idx_controls', self.IDX_CONTROLS_ENABLE)
        rospy.set_param('~idx/idx_controls', self.init_idx_controls)

        rospy.Subscriber('~idx/set_auto_adjust', Bool, self.setAutoAdjustCb, queue_size=1) # start local callback
        self.init_auto_adjust = rospy.get_param('~idx/auto', self.AUTO_ADJUST)
        rospy.set_param('~idx/auto', self.init_auto_adjust)
        self.capabilities_report.has_auto_adjustment = True
        

        rospy.Subscriber('~idx/set_brightness', Float32, self.setBrightnessCb, queue_size=1) # start local callback
        self.init_brightness = rospy.get_param('~idx/brightness', self.BRIGHTNESS_RATIO)
        rospy.set_param('~idx/brightness', self.init_brightness)
        self.capabilities_report.adjustable_brightness = True


        rospy.Subscriber('~idx/set_contrast', Float32, self.setContrastCb, queue_size=1) # start local callback
        self.init_contrast = rospy.get_param('~idx/contrast', self.CONTRAST_RATIO)
        rospy.set_param('~idx/contrast', self.init_contrast)
        self.capabilities_report.adjustable_contrast = True
        

        rospy.Subscriber('~idx/set_thresholding', Float32, self.setThresholdingCb, queue_size=1) # start local callback
        self.init_thresholding = rospy.get_param('~idx/thresholding', self.THRESHOLD_RATIO)
        rospy.set_param('~idx/thresholding', self.init_thresholding)
        self.capabilities_report.adjustable_thresholding = True

        rospy.Subscriber('~idx/set_resolution_mode', UInt8, self.setResolutionModeCb, queue_size=1) # start local callback
        self.init_resolution = rospy.get_param('~idx/resolution_mode', self.RESOLUTION_MODE)
        rospy.set_param('~idx/resolution_mode', self.init_resolution)
        self.capabilities_report.adjustable_resolution = True
               
        rospy.Subscriber('~idx/set_framerate_mode', UInt8, self.setFramerateModeCb, queue_size=1) # start local callback
        self.init_framerate = rospy.get_param('~idx/framerate_mode', self.FRAMERATE_MODE)
        rospy.set_param('~idx/framerate_mode', self.init_framerate)
        self.capabilities_report.adjustable_framerate = True

        if (setRangeCb is not None):        
            rospy.Subscriber('~idx/set_range_window', RangeWindow, self.setRangeCb, queue_size=1)
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
                            resolution_mode = rospy.get_param('~idx/resolution_mode', self.RESOLUTION_MODE) 
                            resolution_ratio = resolution_mode/3
                            [cv2_img,new_res] = nepi_img.adjust_resolution(cv2_img, resolution_ratio)
                        if rospy.get_param('~idx/auto', self.AUTO_ADJUST) is False:
                            if self.setBrightnessCb is None:
                                brightness_ratio = rospy.get_param('~idx/brightness', self.BRIGHTNESS_RATIO)
                                cv2_img = nepi_img.adjust_brightness(cv2_img, brightness_ratio)
                            if self.setContrastCb is None:
                                contrast_ratio = rospy.get_param('~idx/contrast', self.CONTRAST_RATIO)
                                cv2_img = nepi_img.adjust_contrast(cv2_img, contrast_ratio)
                            if self.setThresholdingCb is None:
                                threshold_ratio = rospy.get_param('~idx/thresholding', self.THRESHOLD_RATIO)
                                cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold_ratio)
                        else:
                            if self.setAutoAdjustCb is None:
                                cv2_img = nepi_img.adjust_auto(cv2_img, 0.3)
                            
                        if self.setFramerateModeCb is None:
                            framerate_mode = rospy.get_param('~idx/framerate_mode', self.FRAMERATE_MODE) 
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
                            resolution_mode = rospy.get_param('~idx/resolution_mode', self.RESOLUTION_MODE) 
                            resolution_ratio = resolution_mode/3
                            [cv2_img,new_res] = nepi_img.adjust_resolution(cv2_img, resolution_ratio)
                        if rospy.get_param('~idx/auto', self.AUTO_ADJUST) is False:
                            if self.setBrightnessCb is None:
                                brightness_ratio = rospy.get_param('~idx/brightness', self.BRIGHTNESS_RATIO)
                                cv2_img = nepi_img.adjust_brightness(cv2_img, brightness_ratio)
                            if self.setContrastCb is None:
                                contrast_ratio = rospy.get_param('~idx/contrast', self.CONTRAST_RATIO)
                                cv2_img = nepi_img.adjust_contrast(cv2_img, contrast_ratio)
                            if self.setThresholdingCb is None:
                                threshold_ratio = rospy.get_param('~idx/thresholding', self.THRESHOLD_RATIO)
                                cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold_ratio)
                        else:
                            if self.setAutoAdjustCb is None:
                                cv2_img = nepi_img.adjust_auto(cv2_img, 0.5)
                            
                        if self.setFramerateModeCb is None:
                            framerate_mode = rospy.get_param('~idx/framerate_mode', self.FRAMERATE_MODE) 
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
            settings_msg = nepi_nex.create_msg_data_from_settings(idx_params['settings']) if 'settings' in idx_params else 0
            self.status_msg.settings_states = settings_msg 
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
    
