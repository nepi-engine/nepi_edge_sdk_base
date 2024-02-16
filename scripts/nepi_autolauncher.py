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
import os
import glob
import subprocess
import time
import rosparam
import rospy

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

class NEPIAutolauncher:
  DEFAULT_NODE_NAME = "nepi_autolauncher"

  NEPI_DEFAULT_CFG_PATH = '/opt/nepi/ros/etc/'
  DEVICE_CHECK_INTERVAL_S = 3.0

  # TODO: This is just for testing -- should replace or remove entirely
  DEFAULT_PATH_DEVICE_LIST = [
    {
      'path': '/dev/iqr_pan_tilt',
      'ros_package': 'nepi_edge_sdk_ptx',
      'ros_node_type': 'iqr_ros_pan_tilt_node',
      'ros_node_basename': 'iqr_pan_tilt',
      'ros_params': [],
      'config_file_list': ['dummy_config']
    },
    {
      'path': '/dev/ttyUSB0',
      'ros_package': 'mavros',
      'ros_node_type': 'mavros_node',
      'ros_node_basename': 'mavlink',
      'ros_params': ['_fcu_url:=/dev/ttyUSB0:57600'],
      'config_file_list': ['/opt/ros/noetic/share/mavros/launch/apm_pluginlists.yaml', '/opt/ros/noetic/share/mavros/launch/apm_config.yaml']
    }
  ]

  def __init__(self):
    rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    self.node_name = rospy.get_name().split('/')[-1]

    self.launchedDevices = []

    self.resetFromParamServer() # Set up parameters

    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.updateParamServer, paramsModifiedCallback=self.resetFromParamServer)

    rospy.Timer(rospy.Duration(self.DEVICE_CHECK_INTERVAL_S), self.detectAndManageDevices)
    rospy.spin()

  def updateParamServer(self):
    rospy.set_param('~path_device_list', self.path_detections)

  def resetFromParamServer(self):
    self.path_detections = rospy.get_param('~path_device_list', self.DEFAULT_PATH_DEVICE_LIST)
    
  def detectAndManageDevices(self, _): # Extra arg since this is a rospy Timer callback
    # First, clear the still_present flags so that we can reidentify and kill any that aren't running
    for launched in self.launchedDevices:
      #rospy.loginfo('Debug: Know about ' + str(launched))
      launched['still_present'] = False

    # Look for all the known plug-and-play filesystem paths
    for potential in self.path_detections:
      potential_path = potential['path']

      path_match_list = glob.glob(potential_path)
      for i,path in enumerate(path_match_list):
        # Check if this path is already handled
        already_launched = False
        for launched in self.launchedDevices:
          if launched['path'] == path:
            launched['still_present'] = True 
            already_launched = True   
        
        if already_launched == True:
          continue
             
        # If we get here, this path has not yet been handled
        ros_package = potential['ros_package']
        ros_node_type = potential['ros_node_type']
        ros_node_basename = ros_node_type
        if 'ros_node_basename' in potential:
          ros_node_basename = potential['ros_node_basename']

        specifier = "_" + str(i) if (len(path_match_list) > 1) else ""
        ros_node_name = ros_node_basename + specifier
          
        self.checkLoadConfigFile(node_name=ros_node_name)
        
        # And try to launch the node
        ros_params = [] if 'ros_params' not in potential else potential['ros_params']
        self.startNode(package=ros_package, node_type=ros_node_type, node_name=ros_node_name, dev_path=path, params=ros_params)

    # Now kill off any previously started nodes whose detection has failed
    for i,launched in enumerate(self.launchedDevices):
      if launched['still_present'] is False:
        self.stopAndPurgeNode(node_index=i)

  def startNode(self, package, node_type, node_name, dev_path, params=[]):
    node_namespace = rospy.get_namespace() + node_name
    rospy.loginfo(self.node_name + ": Starting new node " + node_namespace + " (" + package + ":" + node_type + ")")

    # Now start the node via rosrun
    node_run_cmd = ['rosrun', package, node_type, '__name:=' + node_name, '_device_path:=' + dev_path] + params
    p = subprocess.Popen(node_run_cmd)
    if p.poll() is not None:
      rospy.logerr("Failed to start " + node_name + " via " + " ".join(x for x in node_run_cmd) + " (rc =" + str(p.returncode) + ")")
    else:
      self.launchedDevices.append({'node_name': node_name, 
                                   'node_subprocess': p, 
                                   'still_present': True,
                                   'path': dev_path})

  def stopAndPurgeNode(self, node_index):
    if node_index >= len(self.launchedDevices):
      rospy.logwarn(self.node_name + ": Cannot stop node with out-of-bounds index " + str(node_index))
      return
    
    node_dict = self.launchedDevices[node_index]
    rospy.loginfo(self.node_name + ": stopping " + node_dict['node_name'])
    if node_dict['node_subprocess'].poll() is None:
      node_dict['node_subprocess'].terminate()
      terminate_timeout = 3
      node_dead = False
      while (terminate_timeout > 0):
        rospy.loginfo(self.node_name + ": Attempting to terminating " + node_dict['node_name'] + " gracefully")
        time.sleep(1)
        if node_dict['node_subprocess'].poll() is None:
          terminate_timeout -= 1
        else:
          node_dead = True
          break
        
      if not node_dead:
        rospy.logwarn(self.node_name + ": Failed to terminate " + node_dict['node_name'] + " gracefully... attempting to kill")
        # Escalate it
        node_dict['node_subprocess'].kill()
        time.sleep(1)
    else:
      rospy.logwarn(self.node_name + ": " + node_dict['node_name'] + " is not running")    
        
    # And remove it from the list
    self.launchedDevices.pop(node_index)  
    return
  
  def nodeIsRunning(self, node_name):
    for launched in self.launchedDevices:
      if launched['node_name'] == node_name:
        if launched['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list    
    rospy.logwarn(self.node_name + ": cannot check run status of unknown node " + node_name)
    return False
  
  def checkLoadConfigFile(self, node_name):
    config_folder = os.path.join(self.NEPI_DEFAULT_CFG_PATH, node_name)
    if not os.path.isdir(config_folder):
      rospy.logwarn(self.node_name + ': No config folder found for %s... creating one at %s', node_name, config_folder)
      os.makedirs(name = config_folder, mode = 0o775)
      return
    
    config_file = os.path.join(config_folder, node_name + ".yaml")
    node_namespace = rospy.get_namespace() + node_name
    if os.path.exists(config_file):
      rospy.loginfo(self.node_name + ": Loading parameters from " + config_file + " to " + node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_name)
      # Seems programmatic rosparam.load_file is not working at all, so use the command-line version instead
      rosparam_load_cmd = ['rosparam', 'load', config_file, node_namespace]
      subprocess.run(rosparam_load_cmd)
    else:
      rospy.logwarn(self.node_name + ": No config file found for " + node_name + " in " + self.NEPI_DEFAULT_CFG_PATH)
    
if __name__ == '__main__':
  node = NEPIAutolauncher()            

        
      

 