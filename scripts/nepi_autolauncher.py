#!/usr/bin/env python
import os
import glob
import subprocess
import time
import rosparam
import rospy

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

class NEPIAutolauncher:
  DEFAULT_NODE_NAME = "nepi_autolauncher"

  DEFAULT_NEPI_CONFIG_PATH = '/opt/nepi/ros/etc/'
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
          
        ros_node_namespace = rospy.get_namespace() + ros_node_name

        # Define a list of potential config files in priority/override order (low to high): 
        #   1. Fixed values based on node name (without specifier, then with)
        #   2. User overrides (without specifier and then with)
        config_files = [
          os.path.join(self.DEFAULT_NEPI_CONFIG_PATH, ros_node_basename, ros_node_basename + ".yaml") # /opt/nepi/ros/etc/example_node/example_node.yaml
        ]
        if ros_node_name != ros_node_basename:
          config_files.append(os.path.join(self.DEFAULT_NEPI_CONFIG_PATH, ros_node_basename, ros_node_name + ".yaml")) # /opt/nepi/ros/etc/example_node/example_node_0.yaml
        if 'config_file_list' in potential:
          for cfg in potential['config_file_list']:
            if cfg.startswith('/'): # Assume absolute path, so don't append any path
              config_files.append(cfg) # /my/abs/path/my_config.yaml
            else:
              config_files.append(os.path.join(self.DEFAULT_NEPI_CONFIG_PATH, ros_node_basename, cfg)) # /opt/nepi/ros/etc/example_node/my_config.yaml
            if specifier != "": # Also add specifier-defined config files to check
              cfg_base, cfg_extension = os.path.splittext(cfg)
              specifier_cfg = cfg_base + specifier + cfg_extension
              config_files.append(os.path.join(self.DEFAULT_NEPI_CONFIG_PATH, ros_node_basename, specifier_cfg)) # /opt/nepi/ros/etc/example_node/my_config_0.yaml
        
        self.checkLoadConfigFile(fname_specifier_list=config_files, fully_qualified_node_name=ros_node_namespace + '/' + ros_node_name)
        
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
  
  def checkLoadConfigFile(self, fname_specifier_list, fully_qualified_node_name):
    config_file = None
    for fname in fname_specifier_list: # Check/Load highest priority last for overrides
      # Try to find this file at various paths
      #   1. As specified
      #   2. With NEPI config file path prepended
      #   3. TODO?
      current_config = None
      if os.path.exists(fname):
        current_config = fname
      elif os.path.exists(os.path.join(self.DEFAULT_NEPI_CONFIG_PATH, fname)):
        current_config = os.path.join(self.DEFAULT_NEPI_CONFIG_PATH, fname)
      
      # Now load whatever we found
      if current_config is not None:
        config_file = current_config
        rospy.loginfo(self.node_name + ": Loading parameters from " + config_file + " for " + fully_qualified_node_name)
        rosparam.load_file(filename = config_file, default_namespace = fully_qualified_node_name)
    
    if config_file is None:
      rospy.logwarn(self.node_name + ": No eligible config file found for " + fully_qualified_node_name + " from " + str(fname_specifier_list))
    
if __name__ == '__main__':
  node = NEPIAutolauncher()            

        
      

 