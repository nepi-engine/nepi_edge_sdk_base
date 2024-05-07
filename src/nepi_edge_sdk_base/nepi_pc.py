#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI pointcloud utility functions include
# 1) Pointcloud conversion functions
# 2) Pointcloud filter functions
# 3) Pointcloud manipulation functions
# 4) Pointcloud rendering functions
# 5) Pointcloud saving functions


import numpy as np
import ros_numpy
import open3d as o3d
import rospy
import copy
import cv2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped

from nepi_edge_skd_base import open3d_ros_helper as op3rh


###########################################
### Pointcloud conversion functions

def rospc_to_o3dpc(rospc, remove_nans=False):
    """ Convert ROS PointCloud2 to Open3D PointCloud
    
    Args: 
        rospc (sensor.msg.PointCloud2): ROS PointCloud2 message
        remove_nans (bool): If true, ignore the NaN points
    
    Returns: 
        o3dpc (o3d.geometry.PointCloud): Open3D PointCloud
    """
    o3d_pc = o3drh.rospc_to_o3dpc(ros_pc, remove_nans)
    return o3d_pc
    

def o3dpc_to_rospc(o3d_pc, stamp=None, frame_id=None):
    """ convert open3d point cloud to ros point cloud
    Args:
        o3dpc (o3d.geometry.PointCloud): open3d point cloud
        frame_id (string): frame id of ros point cloud header
        stamp (rospy.Time): time stamp of ros point cloud header
    Returns:
        rospc (sensor.msg.PointCloud2): ros point cloud message
    """

    ros_pc = o3drh.o3dpc_to_rospc(o3d_pc, stamp, frame_id)
    return ros_pc
    
    
def rosimg_to_o3dimg(ros_img_msg):
    np_array = ros_numpy.image.image_to_numpy(ros_img_msg)  
    o3d_img = o3d.geometry.Image(np_array)
    return o3d_img

def o3dimg_to_rosimg(o3d_img, stamp=None, frame_id=None):
    np_array = np.asarray(o3d_img)
    ros_img_msg = ros_numpy.image.numpy_to_image(np_array, "bgr8")

    if stamp is None:
        ros_img_msg.header.stamp = rospy.Time.now()
    else:
        ros_img_msg.header.stamp = stamp
    
    if frame_id is not None:
        ros_img_msg.header.frame_id = frame_id
    else:
        ros_img_msg.header.frame_id="map"
    return ros_img_msg

def o3dimg_to_cv2img(o3d_img):
    np_array = np.asarray(o3d_img)
    cv2_image = np_array # cv2 images are just numpy arrays
    return cv2_image

def cv2img_to_o3dimg(cv2_image):
    np_array = np.asarray(cv2img[:,:])
    o3d_img = o3d.geometry.Image(np_array)
    return o3d_img
    
###########################################
### Pointcloud filter functions    

def apply_pass_through_filter(o3d_pc, x_range, y_range, z_range):
    """ apply 3D pass through filter to the open3d point cloud
    Args:
        o3dpc (o3d.geometry.PointCloud): open3d point cloud
        x_range (list): list of [x_min, x_maz]
        y_range (list): list of [y_min, y_maz]
        z_range (list): list of [z_min, z_max]
    Returns:
        o3dpc (o3d.geometry.PointCloud): filtered open3d point cloud
    some codes from https://github.com/powersimmani/example_3d_pass_through-filter_guide
    """
    o3d_pc = o3drh.apply_pass_through_filter(o3d_pc, x_range, y_range, z_range)
    return o3d_pc
    
###########################################
### Pointcloud manipulation functions

def do_transform_point(o3d_pc, transform_stamped):
    """ transform a input cloud with respect to the specific frame
        open3d version of tf2_geometry_msgs.do_transform_point
    Args: 
        o3dpc (o3d.geometry.PointCloud): open3d point cloud
        transform_stamped (geometry_msgs.msgs.TransformStamped): transform to be applied 
    Returns:
        o3dpc (o3d.geometry.PointCloud): transformed open3d point cloud
    """
    o3d_pc = o3drh.do_transform_point(o3d_pc, transform_stamped)
    return o3dpc
    


def crop_with_2dmask(o3d_pc, mask, K=None):
    """ crop open3d point cloud with given 2d binary mask
    Args: 
        o3dpc (o3d.geometry.PointCloud): open3d point cloud
        mask (np.array): binary mask aligned with the point cloud frame shape of [H, W]
        K (np.array): intrinsic matrix of camera shape of (4x4)
        if K is not given, point cloud should be ordered
    Returns:
        o3dpc (o3d.geometry.PointCloud): filtered open3d point cloud
    """
    o3d_pc = o3drh.crop_with_2dmask(o3d_pc, mask, K=None)
    return open3d_pc
    
###########################################
### Pointcloud registration functions

def p2p_icp_registration(source_cloud, target_cloud, n_points=100, threshold=0.02, \
    relative_fitness=1e-10, relative_rmse=1e-8, max_iteration=500, max_correspondence_distance=500):
    """ align the source cloud to the target cloud using point-to-point ICP registration algorithm
    Args: 
        source_cloud (o3d.geometry.PointCloud): source open3d point cloud
        target_cloud (o3d.geometry.PointCloud): target open3d point cloud
        for other parameter, go to http://www.o3d.org/docs/0.9.0/python_api/o3d.registration.registration_icp.html
    Returns:
        icp_result (o3d.registration.RegistrationResult): registration result
    """                        
    [icp_result, evaluation] = o3drh.p2p_icp_registration(source_cloud, target_cloud, n_points, threshold, \
    relative_fitness, relative_rmse, max_iteration, max_correspondence_distance)
    return icp_result, evaluation
            
def ppf_icp_registration(source_cloud, target_cloud, n_points=3000, n_iter=100, tolerance=0.001, num_levels=5, scale=0.001):
    """ align the source cloud to the target cloud using point pair feature (PPF) match
    Args: 
        source_cloud (o3d.geometry.PointCloud): source open3d point cloud
        target_cloud (o3d.geometry.PointCloud): target open3d point cloud
        for other parameter, go to https://docs.opencv.org/master/dc/d9b/classcv_1_1ppf__match__3d_1_1ICP.html
    Returns:
        pose (np.array): 4x4 transformation between source and targe cloud
        residual (float): the output resistration error
    """
    [pose, residual] = o3drh.ppf_icp_registration(source_cloud, target_cloud, n_points=3000, n_iter=100, tolerance=0.001, num_levels=5, scale=0.001)
    return pose, residual



###########################################
### Pointcloud rendering functions

def render_image(o3d_pc,img_width,img_height,background,FOV,center,eye,up):
    render = o3d.visualization.rendering.OffscreenRenderer(img_width, img_height)
    # Set background color
    render.scene.set_background(background)
    # Show the original coordinate axes for comparison.
    # X is red, Y is green and Z is blue.
    render.scene.show_axes(True)
    # Define a simple unlit Material.
    # (The base color does not replace the arrows' own colors.)
    mtl = o3d.visualization.rendering.MaterialRecord()  # or MaterialRecord(), for later versions of Open3D
    mtl.base_color = [1.0, 1.0, 1.0, 1.0]  # RGBA
    mtl.shader = "defaultUnlit"
    render.scene.add_geometry('model',o3d_pc, mtl)
    # Set Lighting
    render.scene.set_lighting(render.scene.LightingProfile.NO_SHADOWS, (0, 0, 0))
    # Optionally set the camera field of view (to zoom in a bit)
    vertical_field_of_view = FOV  
    aspect_ratio = img_width / img_height  # azimuth over elevation
    near_plane = 0.1
    far_plane = 50.0
    fov_type = o3d.visualization.rendering.Camera.FovType.Vertical
    render.scene.camera.set_projection(vertical_field_of_view, aspect_ratio, near_plane, far_plane, fov_type)
    # Look at the origin from the front (along the -Z direction, into the screen), with Y as Up.
    render.scene.camera.look_at(center, eye, up)
    # Render
    o3d_img = render.render_to_image()
    return o3d_img
    
###########################################
### Pointcloud saving functions

def save_o3d_pc(o3d_pc,filename):
    ret = o3d.io.write_point_cloud(filename, o3d_pc)
    return ret

def save_o3d_img(o3d_img,filename):
    ret = o3d.io.write_image(filename, o3d_img)
    return ret
    


