#!/usr/bin/env python
import threading
import rospy
from cv_bridge import CvBridge
import cv2

from std_msgs.msg import UInt8, Float32
from sensor_msgs.msg import Image, PointCloud2

from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow
from nepi_ros_interfaces.srv import IDXCapabilitiesQuery, IDXCapabilitiesQueryResponse

class ROSIDXSensorIF:
    RESOLUTION_MODE_MAX = 3 # LOW, MED, HIGH, ULTRA
    FRAMERATE_MODE_MAX = 3 # LOW, MED, HIGH, ULTRA

    def setResolutionMode(self, msg):
        new_resolution = msg.data
        if (new_resolution > self.RESOLUTION_MODE_MAX):
            rospy.logerr("Resolution mode value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return

        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        status, err_str = self.setResolutionModeCb(new_resolution)
        if status is True:
            rospy.set_param('~idx/resolution_mode', new_resolution)
            self.status_msg.resolution_mode = new_resolution
        else:
            rospy.logerr("Failed to update resolution: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setFramerateMode(self, msg):
        new_framerate = msg.data
        if (new_framerate > self.FRAMERATE_MODE_MAX):
            rospy.logerr("Framerate mode value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return
        
        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        status, err_str = self.setFramerateModeCb(new_framerate)
        if status is True:
            rospy.set_param('~idx/framerate_mode', new_framerate)
            self.status_msg.framerate_mode = new_framerate
        else:
            rospy.logerr("Failed to update framerate: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setContrast(self, msg):
        new_contrast = msg.data
        if (new_contrast < 0.0 and new_contrast != -1.0) or (new_contrast > 1.0):
            rospy.logerr("Contrast value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return

        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        status, err_str = self.setContrastCb(new_contrast)
        if status is True:
            rospy.set_param('~idx/contrast', new_contrast)
            self.status_msg.contrast = new_contrast
        else:
            rospy.logerr("Failed to update contrast: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setBrightness(self, msg):
        new_brightness = msg.data
        if (new_brightness < 0.0 or new_brightness > 1.0):
            rospy.logerr("Brightness value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return

        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        status, err_str = self.setBrightnessCb(new_brightness)
        if status is True:
            rospy.set_param('~idx/brightness', new_brightness)
            self.status_msg.brightness = new_brightness
        else:
            rospy.logerr("Failed to update brightness: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setThresholding(self, msg):
        new_thresholding = msg.data
        if (new_thresholding < 0.0 or new_thresholding > 1.0):
            rospy.logerr("Thresholding value out of bounds")
            self.updateAndPublishStatus(do_updates=False) # No change
            return

        # Call the parent's method and update ROS param as necessary
        # We will only have subscribed if the parent provided a callback at instantiation, so we know it exists here
        status, err_str = self.setThresholdingCb(new_thresholding)
        if status is True:
            rospy.set_param('~idx/thresholding', new_thresholding)
            self.status_msg.thresholding = new_thresholding
        else:
            rospy.logerr("Failed to update thresholding: " + err_str)
        self.updateAndPublishStatus(do_updates=False) # Updated inline here

    def setRange(self, msg):
        pass # TODO

    def setCurrentSettingsAsDefault(self):
        pass # We only use the param server, no member variables to apply to param server

    def updateFromParamServer(self):
        param_dict = rospy.get_param('~idx')
        #rospy.logwarn("Debugging: param_dict = " + str(param_dict))
        if (self.setResolutionModeCb is not None and param_dict.has_key('resolution_mode')):
            self.setResolutionModeCb(param_dict['resolution_mode'])
        if (self.setFramerateModeCb is not None and param_dict.has_key('framerate_mode')):
            self.setFramerateModeCb(param_dict['framerate_mode'])
        if (self.setContrastCb is not None and param_dict.has_key('contrast')):
            self.setContrastCb(param_dict['contrast'])
        if (self.setBrightnessCb is not None and param_dict.has_key('brightness')):
            self.setBrightnessCb(param_dict['brightness'])        
        if (self.setThresholdingCb is not None and param_dict.has_key('thresholding')):
            self.setThresholdingCb(param_dict['thresholding'])
        if (self.setRangeCb is not None and param_dict.has_key('start_range') and param_dict.has_key('stop_range')):
            self.setRangeCb(param_dict['start_range'], param_dict['stop_range'])

        self.updateAndPublishStatus()

    def provide_capabilities(self, _):
        return self.capabilities_report
           
    def __init__(self, setResolutionModeCb=None, setFramerateModeCb=None, setContrastCb=None, 
                 setBrightnessCb=None, setThresholdingCb=None, setRangeCb=None, 
                 getColor2DImgCb=None, stopColor2DImgAcquisitionCb=None, 
                 getGrayscale2DImgCb=None, stopGrayscale2DImgAcquisitionCb=None,
                 getDepthMapCb=None, stopDepthMapAcquisitionCb=None, 
                 getDepthImgCb=None, stopDepthImgAcquisitionCb=None,
                 getPointcloudCb=None, stopPointcloudAcquisitionCb=None, 
                 getPointcloudImgCb=None, stopPointcloudImgAcquisitionCb=None):
        
        data_products = []

        # Create the CV bridge. Do this early so it can be used in the threading run() methods below 
        # TODO: Need one per image output type for thread safety?
        self.cv_bridge = CvBridge()

        self.capabilities_report = IDXCapabilitiesQueryResponse()

        # Set up standard IDX parameters with ROS param and subscriptions
        # Defer actually setting these on the camera via the parent callbacks... the parent may need to do some 
        # additional setup/calculation first. Parent can then get these all applied by calling updateFromParamServer()
        self.setResolutionModeCb = setResolutionModeCb
        if (self.setResolutionModeCb is not None):
            rospy.Subscriber('~idx/set_resolution_mode', UInt8, self.setResolutionMode, queue_size=1)
            init_resolution = rospy.get_param('~idx/resolution_mode', 1) # Default to 1: Medium
            rospy.set_param('~idx/resolution_mode', init_resolution)
            self.capabilities_report.adjustable_resolution = True
        else:
            self.capabilities_report.adjustable_resolution = False
               
        self.setFramerateModeCb = setFramerateModeCb
        if (self.setResolutionModeCb is not None):
            rospy.Subscriber('~idx/set_framerate_mode', UInt8, self.setFramerateMode, queue_size=1)
            init_framerate = rospy.get_param('~idx/framerate_mode', self.FRAMERATE_MODE_MAX) # Default to 3: Ultra
            rospy.set_param('~idx/framerate_mode', init_framerate)
            self.capabilities_report.adjustable_framerate = True
        else:
            self.capabilities_report.adjustable_framerate = False


        self.setContrastCb = setContrastCb
        if (self.setContrastCb is not None):        
            rospy.Subscriber('~idx/set_contrast', Float32, self.setContrast, queue_size=1)
            init_contrast = rospy.get_param('~idx/contrast', 0.5)
            rospy.set_param('~idx/contrast', init_contrast)
            self.capabilities_report.adjustable_contrast = True
        else:
            self.capabilities_report.adjustable_contrast = False
        
        self.setBrightnessCb = setBrightnessCb
        if (self.setBrightnessCb is not None):        
            rospy.Subscriber('~idx/set_brightness', Float32, self.setBrightness, queue_size=1)
            init_brightness = rospy.get_param('~idx/brightness', 0.5)
            rospy.set_param('~idx/brightness', init_brightness)
            self.capabilities_report.adjustable_brightness = True
        else:
            self.capabilities_report.adjustable_brightness = False

        self.setThresholdingCb = setThresholdingCb
        if (self.setThresholdingCb is not None):        
            rospy.Subscriber('~idx/set_thresholding', Float32, self.setThresholding, queue_size=1)
            init_thresholding = rospy.get_param('~idx/thresholding', 0.5)
            rospy.set_param('~idx/thresholding', init_thresholding)
            self.capabilities_report.adjustable_thresholding = True
        else:
            self.capabilities_report.adjustable_thresholding = False

        self.setRangeCb = setRangeCb
        if (self.setRangeCb is not None):        
            rospy.Subscriber('~idx/set_range', RangeWindow, self.setRange, queue_size=1)
            init_start_range = rospy.get_param('~idx/range_window/start_range', 0.0)
            rospy.set_param('~idx/range_window/start_range', init_start_range)
            init_stop_range = rospy.get_param('~idx/range_window/stop_range', 1.0)
            rospy.set_param('~idx/range_window/stop_range', init_stop_range)
            self.capabilities_report.adjustable_range = True
        else:
            self.capabilities_report.adjustable_range = False
        
        # Start the data producer threads
        self.getColor2DImgCb = getColor2DImgCb
        if (self.getColor2DImgCb is not None):
            self.color_img_pub = rospy.Publisher('~idx/color_2d_image', Image, queue_size=1)
            data_products.append('color_2d_image')
            self.color_img_thread = threading.Thread(target=self.runColorImgThread)
            self.color_img_thread.daemon = True # Daemon threads are automatically killed on shutdown
            self.color_img_thread.start()
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
            self.bw_img_thread.start()            
            self.capabilities_report.has_bw_2d_image = True
        else:
            self.capabilities_report.has_bw_2d_image = False
        self.stopGrayscale2DImgCb = stopGrayscale2DImgAcquisitionCb

        self.getDepthMapCb = getDepthMapCb
        if (self.getDepthMapCb is not None):
            self.depth_map_pub = rospy.Publisher('~idx/depth_map', Image, queue_size=1)
            data_products.append('depth_map')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to start depth map thread")
            self.capabilities_report.has_depth_map = True
        else:
            self.capabilities_report.has_depth_map = False
        self.stopDepthMapAcquisitionCb = stopDepthMapAcquisitionCb

        self.getDepthImgCb = getDepthImgCb
        if (self.getDepthImgCb is not None):
            self.depth_img_pub = rospy.Publisher('~idx/depth_image', Image, queue_size = 3)
            data_products.append('depth_image')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to start depth image thread")
            self.capabilities_report.has_depth_image = True
        else:
            self.capabilities_report.has_depth_image = False
        self.stopDepthImgAcquisitionCb = stopDepthImgAcquisitionCb

        self.getPointcloudImgCb = getPointcloudImgCb
        if (self.getPointcloudImgCb is not None):
            self.pointcloud_img_pub = rospy.Publisher('~idx/pointcloud_image', Image, queue_size=1)
            data_products.append('pointcloud_image')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to start pointcloud image thread")
            self.capabilities_report.has_pointcloud_image = True
        else:
            self.capabilities_report.has_pointcloud_image = False
        self.stopPointcloudAcquisitionCb = stopPointcloudAcquisitionCb        

        self.getPointcloudCb = getPointcloudCb
        if (self.getPointcloudCb is not None):
            self.pointcloud_img_pub = rospy.Publisher('~idx/pointcloud', PointCloud2, queue_size=1)
            data_products.append('pointcloud')
            rospy.logwarn("idx_sensor_if.py: TODO: Need to start pointcloud thread")
            self.capabilities_report.has_pointcloud = True
        else:
            self.capabilities_report.has_pointcloud = False
        self.stopPointcloudImgAcquisitionCb = stopPointcloudImgAcquisitionCb   

        # Set up additional publishers
        self.status_msg = IDXStatus()
        self.status_pub = rospy.Publisher('~idx/status', IDXStatus, queue_size=5, latch=True)

        # Set up service providers
        rospy.Service('~idx/capabilities_query', IDXCapabilitiesQuery, self.provide_capabilities)

        # Set up the save data and save cfg i/f
        self.save_data_if = SaveDataIF(data_product_names = data_products)
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer) 

    def runColorImgThread(self):
        rospy.loginfo(rospy.get_name() + ": starting color_2d_image acquisition thread")
        acquiring = False
        while (True):
            if (self.color_img_pub.get_num_connections() > 0):
                acquiring = True
                status, msg, cv_img = self.getColor2DImgCb()
                if (status is False):
                    rospy.logerr_throttle(1, msg)
                    continue
                # Convert cv to ros and publish
                ros_img = self.cv_bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                self.color_img_pub.publish(ros_img)
            elif acquiring is True:
                if self.stopColor2DImgAcquisitionCb is not None:
                    rospy.loginfo(rospy.get_name() + ": stopping color_2d_image acquisition because all subscribers have dropped off")
                    self.stopColor2DImgAcquisitionCb()
                acquiring = False
            else: # No subscribers and already stopped
                rospy.sleep(0.25)

    def runGrayscaleImgThread(self):
        rospy.loginfo(rospy.get_name() + ": starting bw_2d_image acquisition thread")
        acquiring = False
        while (True):
            if (self.grayscale_img_pub.get_num_connections() > 0):
                status, msg, cv_img = self.getGrayscale2DImgCb()
                if (status is False):
                    rospy.logerr_throttle(1, msg)
                    continue

                acquiring = True
                
                # Fix the channel count if necessary
                if cv_img.ndim == 3:
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
                
                # Convert cv to ros and publish
                ros_img = self.cv_bridge.cv2_to_imgmsg(cv_img, encoding="mono8")
                self.grayscale_img_pub.publish(ros_img)
            elif acquiring is True:
                if self.stopGrayscale2DImgCb is not None:
                    rospy.loginfo(rospy.get_name() + ": stopping bw_2d_image acquisition because all subscribers have dropped off")
                    self.stopGrayscale2DImgCb()
                acquiring = False
            else: # No subscribers and already stopped
                rospy.sleep(0.25)

    def updateAndPublishStatus(self, do_updates = True):
        if do_updates is True:
            # TODO: Probably these should be queried from the parent (and through the driver) via explicit callbacks rather than via the param server
            self.status_msg.resolution_mode = rospy.get_param('~idx/resolution_mode', 0)
            self.status_msg.frame_rate_mode = rospy.get_param('~idx/framerate_mode', 0)
            self.status_msg.contrast = rospy.get_param('~idx/contrast', 0)
            self.status_msg.brightness = rospy.get_param('~idx/brightness', 0)
            self.status_msg.thresholding = rospy.get_param('~idx/thresholding', 0)
            self.status_msg.range_window.start_range = rospy.get_param('~idx/range_window/start_range', 0)
            self.status_msg.range_window.stop_range = rospy.get_param('~idx/range_window/stop_range', 0)
        
            # The transfer frame into which 3D data (pointclouds) are transformed for the pointcloud data topic
            self.status_msg.frame_3d = rospy.get_param('~idx/frame_3d', "nepi_center_frame")
        
        self.status_pub.publish(self.status_msg)
    