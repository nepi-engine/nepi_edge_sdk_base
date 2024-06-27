#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI ros utility functions include
# 1) NEPI IDX Driver utility functions

import rospy
from nepi_edge_sdk_base import nepi_img
  

#***************************
# IDX utitlity functions

#Factory Control Values 
DEFAULT_CONTROLS_DICT = dict( controls_enable = True,
    auto_adjust = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.5,
    resolution_mode = 1, # LOW, MED, HIGH, MAX
    framerate_mode = 1, # LOW, MED, HIGH, MAX
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 1.0,
    zoom_ratio = 0.5, 
    rotate_ratio = 0.5,
    frame_3d = 'nepi_center_frame'
    )

def applyIDXControls2Image(cv2_img,IDXcontrols_dict=DEFAULT_CONTROLS_DICT,current_fps=20):
    if IDXcontrols_dict.get("controls_enable"): 
        resolution_ratio = IDXcontrols_dict.get("resolution_mode")/3
        [cv2_img,new_res] = nepi_img.adjust_resolution(cv2_img, resolution_ratio)
        if IDXcontrols_dict.get("auto_adjust") is False:
            cv2_img = nepi_img.adjust_brightness(cv2_img,IDXcontrols_dict.get("brightness_ratio"))
            cv2_img = nepi_img.adjust_contrast(cv2_img,IDXcontrols_dict.get("contrast_ratio"))
            cv2_img = nepi_img.adjust_sharpness(cv2_img,IDXcontrols_dict.get("threshold_ratio"))
        else:
            cv2_img = nepi_img.adjust_auto(cv2_img,0.3)
        ##  Need to get current framerate setting
        ##  Hard Coded for now
        framerate_ratio = IDXcontrols_dict.get("framerate_mode")/3
        [cv2_img,new_rate] = nepi_img.adjust_framerate(cv2_img, current_fps, framerate_ratio)
    return cv2_img

  
