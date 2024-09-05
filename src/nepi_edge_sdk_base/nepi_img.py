#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI image utility functions include
# 1) Image conversion functions
# 2) Image filter functions
# 3) Image manipulation functions
# 4) Image process functions
# 5) Image rendering functions
# 6) Image saving functions


import time
import sys
import rospy
import numpy as np
import ros_numpy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

from nepi_edge_sdk_base import nepi_ros



###########################################
### Image conversion functions

def rosimg_to_cv2img(ros_img_msg, encoding = 'passthrough'):
  """ Convert image from ROS to OpenCV

  Args: 
      ros_img_msg (sensor.msg.Image): ROS Image message

  Returns: 
      cv2_image (cv2.mat): OpenCV Mat Image
  """
  bridge = CvBridge()
  cv2_image = bridge.imgmsg_to_cv2(ros_img_msg, desired_encoding = encoding)
  return cv2_image
    
    
def cv2img_to_rosimg(cv2_image, encoding='passthrough'):
  """ Convert image from OpenCV to ROS

  Args: 
       cv2_image (cv2.mat): OpenCV Mat Image

  Returns: 
      ros_img_msg (sensor.msg.Image): ROS Image message
  """

  bridge = CvBridge()
  ros_img_msg = bridge.cv2_to_imgmsg(cv2_image, encoding = encoding)
  return ros_img_msg
    
###########################################
### Image filter functions    


    
###########################################
### Image manipulation functions


###########################################
### Image process functions
def isgray(cv2_img):
    if len(cv2_img.shape) < 3:
      return True
    else:
      return False

def adjust_auto(cv2_image, sensitivity_ratio = 0.5):    
  # Apply Image Enhancment
  cv2_image.setflags(write=1)
  # Apply threshold filter
  cv2_image=adjust_sharpness(cv2_image, sensitivity_ratio = 0.2)
  #rospy.loginfo("input image shape and type")
  #rospy.loginfo(cv2_image.shape)
  if isgray(cv2_image) is True:
    gray = cv2_image
  else:
    # Color Correction optimization
    Max=[0,0,0]
    for k in range(0, 3):
      Max[k] = np.max(cv2_image[:,:,k])
    Min_Max_channel  = np.min(Max)
    for k in range(0, 3):
      Max_channel  = np.max(cv2_image[:,:,k])
      Min_channel  = np.min(cv2_image[:,:,k])
      Mean_channel = np.mean(cv2_image[:,:,k])
      Chan_scale = (255 - Mean_channel) / (Max_channel - Min_channel)
      if Chan_scale < 1:
        Chan_scale = 1 - (1-Chan_scale)*(255-Min_Max_channel)/170
      elif Chan_scale > 1:
        Chan_scale = 1 + (Chan_scale-1)*(255-Min_Max_channel)/170
      if Chan_scale > 1*(1+sensitivity_ratio):
        Chan_scale = 1 *(1+sensitivity_ratio)
      if Chan_scale < -1*(1+sensitivity_ratio):
        Chan_scale = -1 *(1+sensitivity_ratio)
      Chan_offset = -1*Min_channel
      if Chan_offset < -10 * (1+9*sensitivity_ratio):
        Chan_offset = -10 * (1+9*sensitivity_ratio)
    cv2_image[:,:,k] = (cv2_image[:,:,k] + Chan_offset) * Chan_scale   
    # Contrast and Brightness optimization
    gray = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
  #rospy.loginfo("gray image shape and type")
  #rospy.loginfo(gray.shape)
  # Calculate grayscale histogram
  hist = cv2.calcHist([gray],[0],None,[256],[0,256])
  hist_size = len(hist)
  # Calculate cumulative distribution from the histogram
  accumulator = []
  accumulator.append(float(hist[0]))
  for index in range(1, hist_size):
    accumulator.append(accumulator[index -1] + float(hist[index]))
  # Locate points to clip
  maximum = accumulator[-1]
  clip_hist_percent = (maximum/100.0)
  clip_hist_percent /= 2.0
  # Locate left cut
  minimum_gray = 0
  while accumulator[minimum_gray] < clip_hist_percent:
    minimum_gray += 5
  # Locate right cut
  maximum_gray = hist_size -1
  while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
    maximum_gray -= 1
  # Calculate alpha and beta values
  alpha = 255 / (maximum_gray - minimum_gray) * (sensitivity_ratio*4)
  if alpha>2: ##
    alpha=2 ##
  beta = (-minimum_gray * alpha + 10) * (sensitivity_ratio*4)
  if beta<-50: ##
    beta=-50 ##
  cv2_image = cv2.convertScaleAbs(cv2_image, alpha=alpha, beta=beta)
  return cv2_image
    
def adjust_brightness(cv2_image, sensitivity_ratio = 0.5):
  if sensitivity_ratio != 0.5:
    max_value = 200
    brightness = int((sensitivity_ratio - 0.5) * 2 * max_value) # +- max_value
    if abs(brightness) > max_value:
      brightness = np.sign(brightness) * max_value
    if brightness != 0:
      if brightness > 0:
        shadow = brightness
        highlight = 255
      else:
        shadow = 0
        highlight = 255 + brightness
    alpha = (highlight - shadow)/255
    gamma = shadow
    cv2_image = cv2.addWeighted(cv2_image, alpha, cv2_image, 0, gamma)
  return cv2_image

def adjust_contrast(cv2_image, sensitivity_ratio = 0.5):
  if sensitivity_ratio != 0.5:
    max_value = 90
    contrast = int((sensitivity_ratio - 0.5) * 2 * max_value) # +- max_value
    if abs(contrast) > max_value:
      contrast = np.sign(contrast) * max_value
    f = 131*(contrast + 127)/(127*(131-contrast))
    alpha = f
    gamma = 127*(1-f)
    
    #cv2.convertScaleAbs(cv2_image, cv2_image, alpha_c, gamma_c)
    cv2_image = cv2.addWeighted(cv2_image, alpha, cv2_image, 0, gamma)
  return cv2_image

def adjust_sharpness(cv2_image, sensitivity_ratio = 0.0):
  if sensitivity_ratio != 0.0:
    # gaussian kernel for sharpening
    gaussian_blur = cv2.GaussianBlur(cv2_image,(7,7),sigmaX=2)
    # sharpening using addWeighted()
    sharpness_min = 2
    sharpness_max = 10
    #sharpness = int(sharpness_min + sensitivity_ratio * (sharpness_max-sharpness_min))
    sharpness = int(sharpness_min + sensitivity_ratio**2 * (sharpness_max-sharpness_min))
    cv2_image = cv2.addWeighted(cv2_image,sharpness,gaussian_blur,-(sharpness-1),0)
    #cv2_image * sensitivity_ratio
  return cv2_image

def adjust_resolution(cv2_image, resolution_ratio = 1):
  if resolution_ratio != 1:
    res_scale = 0.3 + 0.7 * resolution_ratio
    new_resolution = (int(cv2_image.shape[0] * res_scale),int(cv2_image.shape[1] * res_scale))
    cv2_image = cv2.resize(cv2_image,(new_resolution), 0, 0, interpolation = cv2.INTER_NEAREST)
  return cv2_image,cv2_image.shape

def adjust_framerate(cv2_image, current_fps, framerate_ratio = 1):
  current_interval_sec = float(1)/current_fps
  delay_sec = 0
  if framerate_ratio != 1:
    delay_sec = current_interval_sec * 10 * (1-framerate_ratio)**3
    nepi_ros.sleep(delay_sec,10)
  new_rate = float(1)/(current_interval_sec + delay_sec)
  return cv2_image, new_rate

def get_contours(cv2_image):
  """ Calculate image contours

  Args: 
       cv2_image (cv2.mat): OpenCV Mat Image

  Returns: 
      contrours3 : OpenCV contours list
      hierarchy3 : 
  """
  #cv2_image.setflags(write=1)
  cv2_mat_gray = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
  ret, thresh2 = cv2.threshold(cv2_mat_gray, 150, 255, cv2.THRESH_BINARY)
  contours3, hierarchy3 = cv2.findContours(thresh2, cv2.RETR_LIST, 
                                       cv2.CHAIN_APPROX_NONE)
  return contours3, hierarchy3

###########################################
### Image overlay functions

def overlay_contours(cv2_image,contours3, color_rgb = (0, 255, 0)):
  #cv2.drawContours(cv2_image, contours3, -1, color_rgb, 2, cv2.LINE_AA)
  return cv2_image
  
def overlay_text(cv2_image, text, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0), scale = 0.5,thickness = 1):
  # Add text overlay
  bottomLeftCornerOfText = (x_px,y_px)
  font                   = cv2.FONT_HERSHEY_SIMPLEX
  lineType               = 1
  cv2.putText(cv2_image,text, 
    bottomLeftCornerOfText, 
    font, 
    scale,
    color_rgb,
    thickness,
    lineType)
  return cv2_image
  
def overlay_text_list(cv2_image, text, x_px = 10 , y_px = 10, line_space_px = 20, color_rgb = (0, 255, 0), scale = 0.5,thickness = 1):
  # Add text overlay
  for text in text_list:
    cv2_overlay_text(cv2_image, text, x_px , y_px, color_rgb, scale, thickness)
    y_px = y_px + line_space_px
    
def overlay_box(cv2_image, color_rgb = (255,255,255), x_px = 10, y_px = 10, w_px = 20, h_px = 20):
      # Add status box overlay
      cv2.rectangle(cv2_image, (x_px, y_px), (w_px, h_px), color_rgb, -1)

    
###########################################
### Image saving functions


    


