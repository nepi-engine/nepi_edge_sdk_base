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
import time
#import usb
import copy
from serial.tools import list_ports


from nepi_edge_sdk_base import nepi_img
  
#***************************
# NEX Driver utility functions

DRIVER_DICT_ENTRY = dict(


)

'''
STANDARD_BUAD_RATES = [110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

SERIAL_PORT_DICT_ENTRY = dict(    vendor_id = 0,
                        product_id = 0,
                        manf_str = "None",
                        buad_rates = []
                   )
                  

def getSerialPortDict():
  port_dict = dict()

  devs = usb.core.find(find_all=True)
  ports = list_ports.comports()
  product_id = 0
  for port in sorted(ports):
    entry = copy.deepcopy(SERIAL_PORT_DICT_ENTRY)
    entry["vender_id"] = port.vid
    for dev in devs:
      if dev.idVendor == port.vid:
        product_id = dev.idProduct
        break
    entry["product_id"] = product_id
    entry["manf_str"] = port.manufacturer
    port_dict[port.device] = entry
  return port_dict

### Function for checking if port is available
def checkSerialPortExists(port_str):
    success = False
    ports = list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      if loc == port_str:
        success = True
    return success
'''


#***************************
# IDX utility functions

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

  
