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
    'class_name': 'chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': '', # Reserved for unique tracking by downstream applications
    'probability': .3, # Probability of detection
    'box_xmin': 10,
    'box_ymin': 10,
    'box_xmax': 100,
    'box_ymax': 100
}

class AiNodeIF:


    def __init__(self,node_name, source_img_topic, pub_sub_namespace, classes_list, setThresholdFunction, processDetectionFunction):

        self.node_name = node_name
        if pub_sub_namespace[-1] == "/":
            pub_sub_namespace = pub_sub_namespace[:-1]
        self.pub_sub_namespace = pub_sub_namespace
        self.setThreshold = setThresholdFunction
        self.processDetection = processDetectionFunction
        self.source_img_topic = source_img_topic

        self.classes_list = classes_list
        self.classes_color_list = self.get_classes_color_list(classes_list)
                 
        self.printMsg("Starting IF setup")
        
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
        	
        self.printMsg("IF Initialization Complete")
        
        
    def printMsg(self,msg, level = 'Info'):
      msg_str = (self.node_name + ": " + str(msg))
      nepi_msg.printMsg(msg_str, level)

        
    def setThresholdCb(self,msg):
        threshold = msg.data
        if (threshold < self.MIN_THRESHOLD):
            threshold = self.MIN_THRESHOLD
        elif (threshold > self.MAX_THRESHOLD):
            threshold = self.MAX_THRESHOLD
        self.setThreshold(threshold)


    def updateDetectionCb(self,source_img_msg):
        ros_img_header = source_img_msg.header
        cv2_source_img = nepi_img.rosimg_to_cv2img(source_img_msg)
        detection_dict_list = self.processDetection(cv2_source_img) 
        self.publishDetectionData(detection_dict_list,ros_img_header)
        # Now create and publish detection image
        if len(detection_dict_list) > 0:
            cv2_detection_img = self.apply_detection_overlay(detection_dict_list,cv2_source_img)
            detection_img_msg = nepi_img.cv2img_to_rosimg(cv2_detection_img, encoding="bgr8")       
        else:
            detection_img_msg = source_img_msg
        self.publishImages(source_img_msg, detection_img_msg)


    def publishDetectionData(self,detection_dict_list,ros_img_header):
        if len(detection_dict_list) > 0:
            bounding_box_msg_list = []
            for detection_dict in detection_dict_list:
                bounding_box_msg = BoundingBox()
                bounding_box_msg.Class = detection_dict['class_name']
                bounding_box_msg.id = detection_dict['id']
                bounding_box_msg.uid = detection_dict['uid']
                bounding_box_msg.probability = detection_dict['probability']
                bounding_box_msg.xmin = detection_dict['box_xmin']
                bounding_box_msg.ymin = detection_dict['box_ymin']
                bounding_box_msg.xmax = detection_dict['box_xmax']
                bounding_box_msg.ymax = detection_dict['box_ymax']
                bounding_box_msg_list.append(bounding_box_msg)
            bounding_boxes_msg = BoundingBoxes()
            bounding_boxes_msg.header.stamp = ros_img_header.stamp
            bounding_boxes_msg.image_header = ros_img_header
            bounding_boxes_msg.image_topic = self.source_img_topic
            bounding_boxes_msg.bounding_boxes = bounding_box_msg_list
            if not rospy.is_shutdown():
                self.bounding_boxes_pub.publish(bounding_boxes_msg)
        found_object_msg = ObjectCount()
        found_object_msg.header.stamp = ros_img_header.stamp
        found_object_msg.count = len(detection_dict_list)
        if not rospy.is_shutdown():
            self.found_object_pub.publish(found_object_msg)


    def publishImages(self,ros_source_img, ros_detection_img):
        if not rospy.is_shutdown():
            self.source_image_pub.publish(ros_source_img)
            self.detection_image_pub.publish(ros_detection_img)

   

    def apply_detection_overlay(self,detection_dict_list,cv2_img):
        for detection_dict in detection_dict_list:
            ###### Apply Image Overlays and Publish Image ROS Message
            # Overlay adjusted detection boxes on image 
            class_name = detection_dict['class_name']
            xmin = detection_dict['box_xmin']
            ymin = detection_dict['box_ymin']
            xmax = detection_dict['box_xmax']
            ymax = detection_dict['box_ymax']
            start_point = (xmin, ymin)
            end_point = (xmax, ymax)


            class_color = (255,0,0)
            if class_name in self.classes_list:
                class_ind = self.classes_list.index(class_name)
                if class_ind < len(self.classes_color_list):
                    class_color = tuple(self.classes_color_list[class_ind])
            img_size = cv2_img.shape
            #self.printMsg("image shape: " + str(img_size),'Warn')
            #self.printMsg("start_point: " + str(start_point),'Warn')
            #self.printMsg("end_point: " + str(end_point),'Warn')
            line_thickness = 2
            if xmax < img_size[0] and ymax < img_size[1]:
                cv2.rectangle(cv2_img, start_point, end_point, color=class_color, thickness=line_thickness)
                # Overlay text data on OpenCV image
                font = cv2.FONT_HERSHEY_DUPLEX
                fontScale, thickness  = self.optimal_font_dims(cv2_img,font_scale = 1.2e-3, thickness_scale = 1.5e-3)
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
                cv2.putText(cv2_img,text2overlay, 
                    bottomLeftCornerOfText, 
                    font, 
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)
        return cv2_img


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