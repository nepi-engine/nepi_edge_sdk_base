#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import sys
import rospy

from sensor_msgs.msg import Image, PointCloud2

from nepi_edge_sdk_base import nepi_pc

DEFAULT_NODE_NAME = 'idx_pointcloud_processor'

class IDXPointcloudProcessor(object):
    # Rendering constants
    Render_Img_Width = 1280
    Render_Img_Height = 720
    Render_Background = [0, 0, 0, 0] # background color rgba
    Render_FOV = 60 # camera field of view in degrees
    Render_Center = [3, 0, 0]  # look_at target
    Render_Eye = [-5, -2.5, 0]  # camera position
    Render_Up = [0, 0, 1]  # camera orientation
    Default_Zoom_Ratio = 0.5  # zoom level: run-time adjustable via ROS parameter

    def __init__(self):
        try:
            self.idx_namespace = rospy.get_param('~idx_namespace') # Throw exception if not present
        except Exception as e:
            rospy.logfatal('Must provide an idx_namespace param to this node... exiting')
            raise e
        
        rospy.loginfo(f"Starting IDX pointcloud processor for namespace {self.idx_namespace}")
        
        # Setup publishers and subscribers
        input_pc_topic = self.idx_namespace + '/pointcloud'
        # Delay subscribing until we have a subscriber ourselves
        input_pc_subscriber = None
        
        output_pc_img_topic = self.idx_namespace + '/pointcloud_image'
        self.output_img_pub = rospy.Publisher(output_pc_img_topic, Image, queue_size=1, tcp_nodelay=True)

        # TODO: Set up a save_data_if for data products

        # Don't use rospy.spin() because we need to check for subscribers intermittently
        r = rospy.Rate(2) # 2Hz
        while not rospy.is_shutdown():
            has_img_subscribers = (self.output_img_pub.get_num_connections() > 0)
            if has_img_subscribers is True: 
                if input_pc_subscriber is None:
                    rospy.loginfo(f'Subscribing to {input_pc_topic} for pointcloud processing')
                    input_pc_subscriber = rospy.Subscriber(input_pc_topic, PointCloud2, self.processPointcloud, queue_size=1)
            else:
                if input_pc_subscriber is not None:
                    rospy.loginfo(f'Unsubscribing from {input_pc_topic} because there are no more processed data subscribers')
                    input_pc_subscriber.unregister()
                    input_pc_subscriber = None
            r.sleep()

    def processPointcloud(self, pc_msg):
        self.publishPointcloudImg(pc_msg)
        # TODO: Other processing?

    def publishPointcloudImg(self, pc_msg):
        ros_timestamp = pc_msg.header.stamp
        ros_frame = pc_msg.header.frame_id

        # Convert to Open3D format
        o3d_pc = nepi_pc.rospc_to_o3dpc(pc_msg, remove_nans=True)
        
        img_width = self.Render_Img_Width
        img_height = self.Render_Img_Height

        controls_enable = rospy.get_param(self.idx_namespace + '/controls_enable', False)
                                        
        if controls_enable is True:
            # No need to do range filtering here, since we are subscribing to the already-range-adjusted idx/pointcloud

            # TODO: Adjust resolution (and possibly downsample for reduced rendering time)
            #       Affects img_width and img_height
                          
            # Adjust Zoom for rendering
            zoom_ratio = rospy.get_param(self.idx_namespace + '/zoom_ratio', self.Default_Zoom_Ratio) # Not much overhead for grabbing this directly from param. server
            zoom_scaler = 1 - zoom_ratio
            render_eye = [number*zoom_scaler for number in self.Render_Eye] # Apply IDX zoom control

            # TODO: Adjust Rotate for rendering
            #       Affects render_eye
        
        else: 
            render_eye = self.Render_Eye
            
        if not rospy.is_shutdown():
            o3d_img = nepi_pc.render_image(o3d_pc,img_width,img_height, self.Render_Background,
                                           self.Render_FOV,self.Render_Center,render_eye,self.Render_Up)
            
            ros_img = nepi_pc.o3dimg_to_rosimg(o3d_img, stamp=ros_timestamp, frame_id=ros_frame)
            self.output_img_pub.publish(ros_img)

if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    node = IDXPointcloudProcessor()

    