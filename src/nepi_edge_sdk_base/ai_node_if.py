#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import rospy
import copy
import time
import numpy as np
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2

from std_msgs.msg import UInt8, Float32, Bool, Empty, String
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import ObjectCount, BoundingBox, BoundingBoxes

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_ais
from nepi_edge_sdk_base import nepi_img


EXAMPLE_DETECTION_DICT_ENTRY = {
    'name': 'chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': '', # Reserved for unique tracking by downstream applications
    'prob': .3, # Probability of detection
    'xmin': 10,
    'ymin': 10,
    'xmax': 100,
    'ymax': 100
}

class AiNodeIF:

    img_width = 0 # Updated on receipt of first image
    img_height = 0 # Updated on receipt of first image
    img_area = 0


    def __init__(self,node_name, source_img_topic, pub_sub_namespace, classes_list, setThresholdFunction, processDetectionFunction):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
        ##############################    


        self.node_name = node_name
        if pub_sub_namespace[-1] == "/":
            pub_sub_namespace = pub_sub_namespace[:-1]
        self.pub_sub_namespace = pub_sub_namespace
        self.setThreshold = setThresholdFunction
        self.processDetection = processDetectionFunction
        self.source_img_topic = source_img_topic

        self.classes_list = classes_list
        self.classes_color_list = self.get_classes_color_list(classes_list)
                 
        nepi_msg.publishMsgInfo(self,"Starting IF setup")
        
        # Create AI Node Publishers

        SOURCE_IMAGE_TOPIC = self.pub_sub_namespace + "/source_image"
        self.source_image_pub = rospy.Publisher(SOURCE_IMAGE_TOPIC, Image, queue_size = 1)
        
        FOUND_OBJECT_TOPIC = self.pub_sub_namespace + "/found_object"
        self.found_object_pub = rospy.Publisher(FOUND_OBJECT_TOPIC, ObjectCount,  queue_size = 1)

        BOUNDING_BOXES_TOPIC = self.pub_sub_namespace + "/bounding_boxes"
        self.bounding_boxes_pub = rospy.Publisher(BOUNDING_BOXES_TOPIC, BoundingBoxes, queue_size = 1)

        DETECTION_IMAGE_TOPIC = self.pub_sub_namespace + "/detection_image"
        self.detection_image_pub = rospy.Publisher(DETECTION_IMAGE_TOPIC, Image,  queue_size = 1)

        time.sleep(1)

        # Create AI Node Subscribers
        THRSHOLD_SUB_TOPIC = self.pub_sub_namespace + '/set_threshold'
        self.set_threshold_sub = rospy.Subscriber(THRSHOLD_SUB_TOPIC, Float32, self.setThresholdCb, queue_size=1)

        IMAGE_SUB_TOPIC = self.source_img_topic
        self.set_threshold_sub = rospy.Subscriber(IMAGE_SUB_TOPIC, Image, self.updateDetectionCb, queue_size=1)
        	
        nepi_msg.publishMsgInfo(self,"IF Initialization Complete")
        
                
    def setThresholdCb(self,msg):
        threshold = msg.data
        if (threshold < self.MIN_THRESHOLD):
            threshold = self.MIN_THRESHOLD
        elif (threshold > self.MAX_THRESHOLD):
            threshold = self.MAX_THRESHOLD
        self.setThreshold(threshold)


    def updateDetectionCb(self,source_img_msg):
        detect_dict_list = None
        ros_img_header = source_img_msg.header
        detect_img_msg = source_img_msg
        cv2_img = nepi_img.rosimg_to_cv2img(source_img_msg)
        cv2_shape = cv2_img.shape
        self.img_width = cv2_shape[1] 
        self.img_height = cv2_shape[0] 
        self.img_area = self.img_height*self.img_width
        try:
            detect_dict_list = self.processDetection(cv2_img) 
            #nepi_msg.publishMsgInfo(self,"AIF got back detect_dict: " + str(detect_dict_list))
            success = True
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to process detection img with exception: " + str(e))

        if detect_dict_list is not None:
            self.publishDetectionData(detect_dict_list,ros_img_header)
            # Now create and publish detection image
            if len(detect_dict_list) > 0:
                #nepi_msg.publishMsgWarn(self,"Starting detect image: " + str(cv2_img.shape))
                cv2_detect_img = self.apply_detection_overlay(detect_dict_list,cv2_img)
                #nepi_msg.publishMsgWarn(self,"Return detect image: " + str(cv2_detect_img.shape))
                detect_img_msg = nepi_img.cv2img_to_rosimg(cv2_detect_img, encoding="bgr8")
            #else:
                #nepi_msg.publishMsgWarn(self,"No detections to add to image")
        else:
            nepi_ros.signal_shutdown("Something went wrong in detection process call")
            nepi_ros.sleep(2)
        self.publishImages(source_img_msg, detect_img_msg)
        

    def publishImages(self,ros_source_img, ros_detect_img):
        if not rospy.is_shutdown():
            self.source_image_pub.publish(ros_source_img)
            self.detection_image_pub.publish(ros_detect_img)

   

    def apply_detection_overlay(self,detect_dict_list,cv2_img):
        cv2_detect_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_detect_img.shape
        if cv2_shape[2] == 1:
            cv2_detect_img = cv2.cvtColor(cv2_detect_img,cv2.COLOR_GRAY2BGR)
        for detect_dict in detect_dict_list:
            area_pixels = (detect_dict['xmax'] - detect_dict['xmin']) * (detect_dict['ymax'] - detect_dict['ymin'])
            if self.img_area > 1:
                area_ratio = area_pixels / self.img_area
            else:
                area_ratio = -999
                
            ###### Apply Image Overlays and Publish Image ROS Message
            # Overlay adjusted detection boxes on image 
            class_name = detect_dict['name']
            xmin = detect_dict['xmin'] + 5
            ymin = detect_dict['ymin'] + 5
            xmax = detect_dict['xmax'] - 5
            ymax = detect_dict['ymax'] - 5
            start_point = (xmin, ymin)
            end_point = (xmax, ymax)


            class_color = (255,0,0)
            
            if class_name in self.classes_list:
                class_ind = self.classes_list.index(class_name)
                if class_ind < len(self.classes_color_list):
                    class_color = tuple(self.classes_color_list[class_ind])
            class_color =  [int(c) for c in class_color]
            img_size = cv2_img.shape[:2]
            line_thickness = 2

            if xmax <= img_size[1] and ymax <= img_size[0]:

                success = False
                try:
                    cv2.rectangle(cv2_detect_img, start_point, end_point, class_color, thickness=line_thickness)
                    success = True
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to create bounding box rectangle: " + str(e))
    
                # Overlay text data on OpenCV image
                if success == True:
                    font = cv2.FONT_HERSHEY_DUPLEX
                    fontScale, thickness  = self.optimal_font_dims(cv2_detect_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
                    fontColor = (0, 255, 0)
                    lineType = 1
                    text_size = cv2.getTextSize("Text", 
                        font, 
                        fontScale,
                        thickness)
                    line_height = text_size[1] * 3
                    # Overlay Label
                    text2overlay=class_name
                    bottomLeftCornerOfText = (xmin + line_thickness,ymin + line_thickness * 2 + line_height)
                    try:
                        cv2_detect_img = cv2.putText(cv2_detect_img,text2overlay, 
                            bottomLeftCornerOfText, 
                            font, 
                            fontScale,
                            fontColor,
                            thickness,
                            lineType)
                    except Exception as e:
                        nepi_msg.publishMsgWarn(self,"Failed to apply overlay text: " + str(e))
            else:
                nepi_msg.publishMsgWarn(self,"xmax or ymax out of range: " + str(img_size)) 
        return cv2_detect_img

    def publishDetectionData(self,detect_dict_list,ros_img_header):
        if len(detect_dict_list) > 0:
            bounding_box_msg_list = []
            for detect_dict in detect_dict_list:
                bounding_box_msg = BoundingBox()
                bounding_box_msg.Class = detect_dict['name']
                bounding_box_msg.id = detect_dict['id']
                bounding_box_msg.uid = detect_dict['uid']
                bounding_box_msg.probability = detect_dict['prob']
                bounding_box_msg.xmin = detect_dict['xmin']
                bounding_box_msg.ymin = detect_dict['ymin']
                bounding_box_msg.xmax = detect_dict['xmax']
                bounding_box_msg.ymax = detect_dict['ymax']
                bounding_box_msg.area_pixels = area_pixels
                bounding_box_msg.area_ratio = area_ratio
                bounding_box_msg_list.append(bounding_box_msg)
            bounding_boxes_msg = BoundingBoxes()
            bounding_boxes_msg.header.stamp = ros_img_header.stamp
            bounding_boxes_msg.image_header = ros_img_header
            bounding_boxes_msg.image_topic = self.source_img_topic
            bounding_boxes_msg.image_width = cv2_shape[1]
            bounding_boxes_msg.image_height = cv2_shape[0]
            bounding_boxes_msg.bounding_boxes = bounding_box_msg_list
            if not rospy.is_shutdown():
                self.bounding_boxes_pub.publish(bounding_boxes_msg)
        found_object_msg = ObjectCount()
        found_object_msg.header.stamp = ros_img_header.stamp
        found_object_msg.count = len(detect_dict_list)
        if not rospy.is_shutdown():
            self.found_object_pub.publish(found_object_msg)



    def optimal_font_dims(self, img, font_scale = 2e-3, thickness_scale = 5e-3):
        h, w, _ = img.shape
        font_scale = min(w, h) * font_scale
        thickness = math.ceil(min(w, h) * thickness_scale)
        return font_scale, thickness
                    

    def get_classes_color_list(self,classes_str_list):
        rgb_list = []
        if len(classes_str_list) > 0:
            cmap = plt.get_cmap('viridis')
            color_list = cmap(np.linspace(0, 1, len(classes_str_list))).tolist()
            for color in color_list:
                rgb = []
                for i in range(3):
                    rgb.append(int(color[i]*255))
                rgb_list.append(rgb)
        return rgb_list