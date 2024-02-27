#!/usr/bin/env python
#
# NEPI Dual-Use License
# Project: nepi_edge_sdk_base
#
# This license applies to any user of NEPI Engine software
#
# Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
# see https://github.com/numurus-nepi/nepi_edge_sdk_base
#
# This software is dual-licensed under the terms of either a NEPI software developer license
# or a NEPI software commercial license.
#
# The terms of both the NEPI software developer and commercial licenses
# can be found at: www.numurus.com/licensing-nepi-engine
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - https://www.numurus.com/licensing-nepi-engine
# - mailto:nepi@numurus.com
#
#

# NEPI RBS Auto Discovery Script for Mavlink devices

import rospy
import serial
import serial.tools.list_ports
import subprocess

#Define Discovery Search Parameters
BAUDRATE_LIST = [57600] # Just one supported baud rate at present

# Globals
mavlink_node_list = []
mavlink_subproc_list = []
mavlink_port_list = []

#########################################
# Mavlink Discover Method
#########################################

### Function to try and connect to device and also monitor and clean up previously connected devices
def mavlink_discover(active_port_list):
  global mavlink_node_list
  global mavlink_subproc_list
  global mavlink_port_list

  node_name = rospy.get_name().split('/')[-1]
  # Find serial ports
  rospy.logdebug(node_name + ": Looking for serial ports on device")
  port_list = []
  ports = serial.tools.list_ports.comports()
  for loc, desc, hwid in sorted(ports):
    rospy.logdebug(node_name + ": Found serial_port at: " + loc)
    port_list.append(loc)
  # Checking for devices on available serial ports
  for port_str in port_list:
    if port_str not in active_port_list:
      found_heartbeat = False
      for baud_int in BAUDRATE_LIST:
        rospy.loginfo(node_name + ": Connecting to serial port " + port_str + " with baudrate: " + str(baud_int))
        try:
          # Try and open serial port
          rospy.loginfo(node_name + ": Opening serial port " + port_str + " with baudrate: " + str(baud_int))
          serial_port = serial.Serial(port_str,baud_int,timeout = 1)
        except Exception as e:
          rospy.logwarn(node_name + ": Unable to open serial port " + port_str + " with baudrate: " + str(baud_int) + "(" + str(e) + ")")
          continue
        
        for i in range(0,64): # Read up to 64 packets waiting for heartbeat
          try:
            #serial_port.read_until(b'\xFE', 255) # This is the MAVLINK_1 magic number
            serial_port.read_until(b'\xFD', 255) # MAVLINK_2 packet start magic number
          except Exception as e:
            print("read_until() failed (" + str(e) + ")")
            continue
          try:
            pkt_hdr = serial_port.read(9) # MAVLINK_2 packet header length
          except Exception as e:
            print("read failed (" + str(e) + ")")
            continue

          # Initialize as a non-heartbeat packet
          pkt_len = 255
          sys_id = 255
          comp_id = 255
          msg_id_l = 255

          if len(pkt_hdr) == 9:
            #print(''.join('{:02x}'.format(x) for x in pkt_hdr))
            # This decoding assumes mavlink_2 format packet
            pkt_len = pkt_hdr[0]
            sys_id = pkt_hdr[4]
            comp_id = pkt_hdr[5]
            msg_id_l, msg_id_m, msg_id_h = pkt_hdr[6], pkt_hdr[7], pkt_hdr[8]
          
          # Identify a heartbeat packet by tell-tale signs
          if pkt_len == 9 and msg_id_l == 0x0 and msg_id_m == 0x0 and msg_id_h == 0x0: # Heartbeat message id = 0x00 00 00
            print(str(i) + ": HTBT, sys_id = " + str(sys_id) + ', comp_id = ' + str(comp_id))
            found_heartbeat = True
            break
          
        # Clean up the serial port
        rospy.loginfo(node_name + ": Closing serial port " + port_str)
        serial_port.close()
        
        # If this is a mavlink, load params and launch the mavros node
        if found_heartbeat:
          addr_str = str(sys_id)
          rospy.loginfo(node_name + ": Found mavlink device at: " + addr_str)
          port_str_short = port_str.split('/')[-1]
          mavlink_node_name = "mavlink_" + port_str_short + "_" + addr_str
          mavlink_node_namespace = rospy.get_namespace() + mavlink_node_name
          
          # Load the proper configs for APM
          rosparam_load_cmd = ['rosparam', 'load', '/opt/ros/noetic/share/mavros/launch/apm_pluginlists.yaml', mavlink_node_namespace]
          subprocess.run(rosparam_load_cmd)

          rosparam_load_cmd = ['rosparam', 'load', '/opt/ros/noetic/share/mavros/launch/apm_config.yaml', mavlink_node_namespace]
          subprocess.run(rosparam_load_cmd)
        
          fcu_url = port_str + ':' + str(baud_int)
          node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + mavlink_node_name, '_fcu_url:=' + fcu_url] 
          p = subprocess.Popen(node_run_cmd)
          active_port_list.append(port_str)
          mavlink_port_list.append(port_str)
          mavlink_node_list.append(mavlink_node_name)
          mavlink_subproc_list.append(p)
          
          break # Don't check any more baud rates since this one was already successful
  
  # Finally check for and purge any nodes no longer running
  for i,node in enumerate(mavlink_node_list):
    subproc = mavlink_subproc_list[i]
    poll_ret = subproc.poll()
    if poll_ret is not None:
      rospy.logwarn(node + ": mavros process no longer running... cleaning up resources")
      port = mavlink_port_list[i]
      if port not in active_port_list:
        rospy.logwarn(node + ": port " + port + " unexpectedly already inactivated")
      else:
        active_port_list.remove(port)

      # Clean up the globals  
      del mavlink_port_list[i]
      del mavlink_node_list[i]
      del mavlink_subproc_list[i]
  
  return active_port_list
