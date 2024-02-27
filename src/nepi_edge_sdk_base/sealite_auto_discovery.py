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

# NEPI Auto Discovery Script for Sealite devices

import rospy
import serial
import serial.tools.list_ports
import subprocess

#Define Discovery Search Parameters
#Define Discovery Search Parameters
BAUDRATE_LIST = [9600,19200,57600] # Three supported baud rates
ADDRESS_LIST = list(range(1,10))  # Total range 1-255

# Globals
sealite_node_list = []
sealite_subproc_list = []
sealite_port_list = []

#########################################
# Sealite Discover Method
#########################################

### Function to try and connect to device and also monitor and clean up previously connected devices
def sealite_discover(active_port_list):
  global sealite_node_list
  global sealite_subproc_list
  global sealite_port_list

  node_name = rospy.get_name().split('/')[-1]
  base_namespace = rospy.get_namespace()
  # Find serial ports
  rospy.logdebug(node_name + ": Looking for serial ports on device")
  port_list = []
  ports = serial.tools.list_ports.comports()
  for loc, desc, hwid in sorted(ports):
    rospy.logdebug(node_name + ": Found serial_port at: " + loc)
    port_list.append(loc)

  for port_str in port_list:
    if port_str not in active_port_list:
      found_sealite = False
      for baud_int in BAUDRATE_LIST:
        if found_sealite:
          break # No need to search more baud rates
        rospy.loginfo(node_name + ": Connecting to serial port " + port_str + " with baudrate: " + str(baud_int))
        try:
          # Try and open serial port
          rospy.loginfo(node_name + ": Opening serial port " + port_str + " with baudrate: " + str(baud_int))
          serial_port = serial.Serial(port_str,baud_int,timeout = 1)
        except Exception as e:
          rospy.logwarn(node_name + ": Unable to open serial port " + port_str + " with baudrate: " + str(baud_int) + "(" + str(e) + ")")
          continue

        for addr in ADDRESS_LIST:
          addr_str = str(addr)
          zero_prefix_len = 3-len(addr_str)
          for z in range(zero_prefix_len):
            addr_str = ('0' + addr_str)
          # Create message string
          ser_msg= ('!' + addr_str + ':INFO?')
          ser_str = (ser_msg + '\r\n')
          # Send Serial String
          #print("")
          #print("Sending serial message: " + ser_msg)
          b=bytearray()
          b.extend(map(ord, ser_str))
          serial_port.write(b)
          #print("Waiting for response")
          rospy.sleep(.005)
          bs = serial_port.readline()
          response = bs.decode()
          if len(response) > 2:
            rospy.loginfo(node_name + ": Got response: " + response)
            if response[3] == ',':
              addr_str = response[0:3]
              try:
                addr_int = int(addr)
                rospy.loginfo(node_name + ": Found device at address: " + addr_str)
                found_sealite = True
                break # Don't check any more addresses
              except Exception as a:
                rospy.logwarn(node_name + ": Returned device message not valid (" + str(a) + ")")
          
        # Clean up the serial port
        rospy.loginfo(node_name + ": Closing serial port " + port_str)
        serial_port.close()
        
        # If this is a sealite, load params and launch the mavros node
        if found_sealite:
          addr_str = str(addr_int)
          rospy.loginfo("%s: Found sealite device %s at %s:%d", node_name, addr_str, port_str, baud_int)
          port_str_short = port_str.split('/')[-1]
          sealite_node_name = "sealite_" + port_str_short + "_" + addr_str
          sealite_node_namespace = base_namespace + sealite_node_name
          
          rospy.set_param(sealite_node_namespace + sealite_node_name + '/port_str', port_str)
          rospy.set_param(sealite_node_namespace + sealite_node_name + '/baud_int', baud_int)
          rospy.set_param(sealite_node_namespace + sealite_node_name + '/addr_str', addr_str)
          rospy.sleep(2)
          # Pass the name as a regular cmd-line arg since we can't rosrun this new node as it is not currently installed in ROS path
          node_run_cmd = ['rosrun', 'nepi_edge_sdk_lsx', 'sealite_lsx_driver_script.py', '__name:=' + sealite_node_name] 
          p = subprocess.Popen(node_run_cmd)
          sealite_node_list.append(sealite_node_name)
          sealite_subproc_list.append(p)
          sealite_port_list.append(port_str)
          active_port_list.append(port_str) # Add a new entry for the parent's master list
          break # Don't check any more baud rates since this one was already successful
  
  # Finally check for and purge any nodes no longer running
  topic_list=rospy.get_published_topics(namespace='/')
  for i,node in enumerate(sealite_node_list):
    full_node_name = base_namespace + node
    check_topic_name = (full_node_name + "/lsx/active")
    rospy.logdebug(node_name + ": Checking for topic name: " + check_topic_name)
    
    node_exists = False
    for topic_entry in topic_list:
      if topic_entry[0].find(check_topic_name) == -1:
        rospy.logwarn(node + ": sealite node no longer responding on \"active\" topic... cleaning up resources")
        port_to_free = sealite_port_list(i)
        if port_to_free not in active_port_list:
          rospy.logwarn(node + ": port " + port + " unexpectedly already inactivated")
        else:
          active_port_list.remove(port_to_free)
        
        # Clean up the globals  
        del sealite_port_list[i]
        del sealite_node_list[i]
        del sealite_subproc_list[i]
  
  return active_port_list
