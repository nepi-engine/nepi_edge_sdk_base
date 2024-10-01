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

from std_msgs.msg import UInt8, Float32, Bool, Empty, String
from nepi_ros_interfaces.msg import ObjectCount, BoundingBox, BoundingBoxes

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img

NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()

EXAMPLE_DETECTION_DICT_ENTRY = {
    'Class': 'Chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': 0, # Reserved for unique tracking by downstream applications
    'probability': .3, # Probability of detection
    'box_xmin': 10,
    'box_ymin': 100,
    'box_xmax': 10,
    'box_ymax': 100
}


class AiNodeIF:

    def __init__(self):
                 
        self.node_name = rospy.get_name()
        rospy.loginfo(self.node_name + ": Starting IF setup")
        # Create a node msg publisher
        self.msg_pub = rospy.Publisher("~messages", String, queue_size=1)
        time.sleep(1)
        
        # Create AI Node Publishers and Subscribers
        self.source_image_topic = source_image_topic
        SOURCE_IMAGE_TOPIC = NEPI_BASE_NAMESPACE + "ai_detector_mgr/source_image"
        self.source_image_pub = rospy.Publisher(SOURCE_IMAGE_TOPIC, Image, self.depthMapCb, queue_size = 1)
        
        FOUND_OBJECT_TOPIC = NEPI_BASE_NAMESPACE + "ai_detector_mgr/found_object"
        self.found_object_pub = rospy.Publisher(FOUND_OBJECT_TOPIC, ObjectCount,  queue_size = 1)

        BOUNDING_BOXES_TOPIC = NEPI_BASE_NAMESPACE + "ai_detector_mgr/bounding_boxes"
        self.bounding_boxes_pub = rospy.Publisher(BOUNDING_BOXES_TOPIC, BoundingBoxes, queue_size = 1)

        DETECTION_IMAGE_TOPIC = NEPI_BASE_NAMESPACE + "ai_detector_mgr/detection_image"
        self.detection_image_pub = rospy.Publisher(DETECTION_IMAGE_TOPIC, Image,  queue_size = 1)
        	
        self.publishMsg("IF Initialization Complete")
        
        
    def publishMsg(self,msg):
      msg_str = (self.node_name + ": " + str(msg))
      rospy.loginfo(msg_str)
      self.msg_pub.publish(msg_str)
        

    def get_classes_color_list(self,classes_str_list):
        rgb_list = []
        if len(classes_str_list) > 0:
            cmap = plt.get_cmap('viridis')
            color_list = cmap(np.linspace(0, 1, len(classes_str_list))).tolist()
            for color in color_list:
                for i in range(3):
                    rgb.append(int(color[i]*255))
                rgb_list.append(rgb)
        return rgb_list


    def publishDetectionData(self,detection_dict_list):
        found_object_msg = ObjectCount()
        found_object_msg.header.stamp = process_time
        found_object_msg.count = len(detection_dict_list)
        if not rospy.is_shutdown():
            self.found_object_pub.publish(found_object_msg)

        if len(detection_dict_list) > 0:
            bounding_box_msg_list = []
            for detection_dict in detection_dict_list:
                bounding_box_msg = BoundingBox()
                bounding_box_msg.Class = detection_dict['Class']
                bounding_box_msg.id = detection_dict['id']
                bounding_box_msg.uid = detection_dict['uid']
                bounding_box_msg.probability = detection_dict['probability']
                bounding_box_msg.xmin = detection_dict['box_xmin']
                bounding_box_msg.ymin = detection_dict['box_ymin']
                bounding_box_msg.xmax = detection_dict['box_xmax']
                bounding_box_msg.ymax = detection_dict['box_ymax']
                bounding_box_msg_list.append(bounding_box_msg)
            bounding_boxes_msg = BoundingBoxes()
            bounding_boxes_msg.header.stamp = process_time
            bounding_boxes_msg.image_header = image_header
            bounding_boxes_msg.image_topic = self.source_image_topic
            bounding_boxes_msg.bounding_boxes = bounding_box_msg_list
            if not rospy.is_shutdown():
                self.bounding_boxes_pub.publish(bounding_boxes_msg)
                

