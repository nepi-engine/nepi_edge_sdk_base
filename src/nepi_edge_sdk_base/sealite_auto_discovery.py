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

from nepi_ros_interfaces.srv import LSXCapabilitiesQuery

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

  node_name = rospy.get_name().split('/')[-1] + '/sealite_auto_discovery'
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
        rospy.logdebug(node_name + ": Connecting to serial port " + port_str + " with baudrate: " + str(baud_int))
        try:
          # Try and open serial port
          rospy.logdebug(node_name + ": Opening serial port " + port_str + " with baudrate: " + str(baud_int))
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
          try:
            serial_port.write(b)
            #print("Waiting for response")
            rospy.sleep(.005)
            bs = serial_port.readline()
          except Exception as e:
            rospy.logerr('%s: Got a serial read/write error (%s)', node_name, str(e))
            break
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
        rospy.logdebug(node_name + ": Closing serial port " + port_str)
        serial_port.close()
        
        # If this is a sealite, load params and launch the mavros node
        if found_sealite:
          addr_str = str(addr_int)
          zero_prefix_len = 3-len(addr_str)
          for z in range(zero_prefix_len):
            addr_str = ('0' + addr_str)
          rospy.loginfo("%s: Found sealite device %s at %s:%d", node_name, addr_str, port_str, baud_int)
          port_str_short = port_str.split('/')[-1]
          sealite_node_name = "sealite_" + port_str_short + "_" + addr_str
          sealite_node_namespace = base_namespace + sealite_node_name
          
          rospy.set_param(sealite_node_namespace + '/port_str', port_str)
          rospy.set_param(sealite_node_namespace + '/baud_int', baud_int)
          rospy.set_param(sealite_node_namespace + '/addr_str', addr_str)
          rospy.sleep(2)
          # Pass the name as a regular cmd-line arg since we can't rosrun this new node as it is not currently installed in ROS path
          node_run_cmd = ['rosrun', 'nepi_edge_sdk_lsx', 'sealite_lsx_driver_script.py', '__name:=' + sealite_node_name] 
          p = subprocess.Popen(node_run_cmd)

          # And make sure it actually starts up fully by waiting for a guaranteed service
          capabilities_service_name = sealite_node_namespace + '/lsx/capabilities_query'
          try:
            rospy.wait_for_service(capabilities_service_name, timeout=10) # TODO: 10 seconds always sufficient for the driver?

            # No exception, all good
            sealite_node_list.append(sealite_node_name)
            sealite_subproc_list.append(p)
            sealite_port_list.append(port_str)
            active_port_list.append(port_str) # Add a new entry for the parent's master list
            break # Don't check any more baud rates since this one was already successful
          except:
            rospy.logerr("%s: Failed to start %s", node_name, sealite_node_name)
  
  # Finally check for and purge any nodes no longer running or nodes associated with detached hardware
  topic_list=rospy.get_published_topics(namespace='/')
  for i,node in enumerate(sealite_node_list):
    purge_node = False

    full_node_name = base_namespace + node
    node_port = sealite_port_list[i]
    node_subproc = sealite_subproc_list[i]
    # Check that the node process is still running
    if node_subproc.poll() is not None: # process dead
      rospy.logwarn('%s: Node process for %s is no longer running... purging from managed list', node_name, node)
      purge_node = True
    # Check that the node's port still exists
    elif node_port not in active_port_list:
      rospy.logwarn('%s: Port %s associated with node %s no longer detected', node_name, node_port, node)
      purge_node = True
    else:
      # Now check that the node is actually responsive
      # Use a service call so that we can provide are assured of synchronous response
      capabilities_service_name = full_node_name + '/lsx/capabilities_query'
      capabilities_query = rospy.ServiceProxy(capabilities_service_name, LSXCapabilitiesQuery)
      try:
        # We don't actually care about the contents of the response at this point, but we might in the future for
        # additional aliveness check logic:
        #response = capability_service()
        capabilities_query()
      except: # Any exception indicates that the service call failed
        rospy.logwarn('%s: Node %s is no longer responding to capabilities queries')
        purge_node = True

    if purge_node:
      rospy.logwarn('%s: Purging node %s', node_name, node)

      if node_port in active_port_list:
        rospy.logwarn('%s: Removing port %s from active list as part of node purging', node_name, node_port)
        active_port_list.remove(node_port)

      if node_subproc.poll() is None: # Still alive
        rospy.logwarn('%s: Issuing sigterm to process for %s as part of node purging', node_name, node)
        node_subproc.kill() # Hard kill
        # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
        rospy.sleep(10) # Long enough for process to die and rosnode cleanup to see the node as disconnected
        cleanup_proc = subprocess.Popen(['rosnode', 'cleanup'], stdin=subprocess.PIPE)
        try:
          cleanup_proc.communicate(input=bytes("y\r\n", 'utf-8'), timeout=10)
          cleanup_proc.wait(timeout=10) 
        except Exception as e:
          rospy.logwarn('%s: rosnode cleanup failed (%s)', node_name, str(e))

      # Clean up the globals  
      del sealite_port_list[i]
      del sealite_node_list[i]
      del sealite_subproc_list[i]
 
  return active_port_list
