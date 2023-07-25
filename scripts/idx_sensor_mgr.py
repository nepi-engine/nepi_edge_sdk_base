#!/usr/bin/env python
import sys
import os
import subprocess
import time
import roslaunch
import rosparam
import rospy

class IDXSensorMgr:
  DEFAULT_NODE_NAME = "idx_sensor_mgr"

  NEPI_IDX_SENSOR_PARAM_PATH = '/opt/nepi/ros/etc/idx_sensors/'
  V4L2_SENSOR_CHECK_INTERVAL_S = 3.0

  def __init__(self):
    # Launch the ROS node
    rospy.loginfo("Starting " + self.DEFAULT_NODE_NAME)
    rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    self.node_name = rospy.get_name().split('/')[-1]

    self.sensorList = []

    rospy.Timer(rospy.Duration(self.V4L2_SENSOR_CHECK_INTERVAL_S), self.detectAndManageV4L2Sensors)
    rospy.spin()
    
  def detectAndManageV4L2Sensors(self, _): # Extra arg since this is a rospy Timer callback
    # First grab the current list of known V4L2 devices
    p = subprocess.Popen(['v4l2-ctl', '--list-devices'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    stdout,_ = p.communicate()
    #if p.returncode != 0:  # This can happen because there are no longer any video devices connected, so v4l2-ctl returns error
      #raise Exception("Failed to list v4l2 devices: " + stdout)
    out = stdout.splitlines()

    tmp_device_type = None
    tmp_device_path = None
    active_paths = list()
    nLines = len(out)
    for i in range(0, nLines):
      line = out[i].strip()
      if line.endswith(':'):
        tmp_device_type = line.split('(')[0].strip()
      elif (tmp_device_type != None) and (line.startswith('/dev/video')):
        tmp_device_path = line
        active_paths.append(tmp_device_path) # To check later that the sensor list has no entries for paths that have disappeared
        known_sensor = False
        # Check if this sensor is already known and launched
        for sensor in self.sensorList:
          if sensor['sensor_class'] != 'v4l2':
            continue

          if sensor['device_path'] == tmp_device_path:
            known_sensor = True
            if sensor['device_type'] != tmp_device_type:
              # Uh oh -- sensor has switched on us!
              # Kill previous and start new?
              rospy.logwarn(self.node_name + ": detected V4L2 device type change (" + sensor['device_type'] + "-->" + 
                            tmp_device_type + ") for device at " + tmp_device_path)
              self.stopAndPurgeSensorNode(sensor['node_namespace'])
              self.startV4L2SensorNode(type = tmp_device_type, path = tmp_device_path)
            elif not self.sensorNodeIsRunning(sensor['node_namespace']):
              rospy.logwarn(self.node_name + ": node " + sensor['node_name'] + " is not running. Restarting")
              self.stopAndPurgeSensorNode(sensor['node_namespace'])
              self.startV4L2SensorNode(type = tmp_device_type, path = tmp_device_path)
            break

        if not known_sensor:
          self.startV4L2SensorNode(type = tmp_device_type, path = tmp_device_path)

    # Handle sensors which no longer have a valid active V4L2 device path
    for sensor in self.sensorList:
      if sensor['sensor_class'] != 'v4l2':
        continue

      if sensor['device_path'] not in active_paths:
        rospy.logwarn(self.node_name + ': ' + sensor['node_namespace'] + ' path ' + sensor['device_path'] + ' no longer exists... sensor disconnected?')
        self.stopAndPurgeSensorNode(sensor['node_namespace'])


  def startV4L2SensorNode(self, type, path):
    # First, get a unique name
    root_name = type.replace(' ','_').lower()
    same_type_count = 0
    for sensor in self.sensorList:
      if sensor['device_type'] == type:
        same_type_count += 1

    sensor_node_name = root_name
    if same_type_count > 0:
      sensor_node_name += '_' + str(same_type_count)

    sensor_node_namespace = rospy.get_namespace() + sensor_node_name
    rospy.loginfo(self.node_name + ": Initiating new V4L2 node " + sensor_node_namespace)

    self.checkLoadConfigFile(fname_specifier_list = [sensor_node_name, root_name, 'v4l2_generic'], node_namespace = sensor_node_namespace)

    # Now start the node via rosrun
    # rosrun nepi_edge_sdk_v4l2 v4l2_camera_node.py __name:=usb_cam_1 _device_path:=/dev/video0
    sensor_node_run_cmd = ['rosrun', 'nepi_edge_sdk_v4l2', 'v4l2_camera_node.py', '__name:=' + sensor_node_name, '_device_path:='+path]
    p = subprocess.Popen(sensor_node_run_cmd)
    if p.poll() is not None:
      rospy.logerr("Failed to start " + sensor_node_name + " via " + " ".join(x for x in sensor_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
    else:
      self.sensorList.append({'sensor_class': 'v4l2', 'device_path': path, 'device_type': type, 
                              'node_name': sensor_node_name, 'node_namespace': sensor_node_namespace,
                              'node_subprocess': p})

  def stopAndPurgeSensorNode(self, node_namespace):
    rospy.loginfo(self.node_name + ": stopping " + node_namespace)
    for i, sensor in enumerate(self.sensorList):
      if sensor['node_namespace'] == node_namespace:
        if sensor['node_subprocess'].poll() is None:
          sensor['node_subprocess'].terminate()
          terminate_timeout = 3
          node_dead = False
          while (terminate_timeout > 0):
            time.sleep(1)
            if sensor['node_subprocess'].poll() is None:
              terminate_timeout -= 1
            else:
              node_dead = True
              break
          if not node_dead:
            # Escalate it
            sensor['node_subprocess'].kill()
            time.sleep(1)
        
        # And remove it from the list
        self.sensorList.pop(i)  
        return
    
    rospy.logwarn(self.node_name + ": Unable to stop unknown node " + node_namespace)

  def sensorNodeIsRunning(self, node_namespace):
    for sensor in self.sensorList:
      if sensor['node_namespace'] == node_namespace:
        if sensor['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list    
    rospy.logwarn(self.node_name + ": cannot check run status of unknown node " + node_namespace)
    return False
  
  def checkLoadConfigFile(self, fname_specifier_list, node_namespace):
    config_file = None
    for name in fname_specifier_list:
      if os.path.exists(os.path.join(self.NEPI_IDX_SENSOR_PARAM_PATH, name + ".yaml")):
        config_file = os.path.join(self.NEPI_IDX_SENSOR_PARAM_PATH, name + ".yaml")

    if config_file:
      rospy.loginfo(self.node_name + ": Loading parameters from " + config_file + " for " + node_namespace)
      rosparam.load_file(filename = config_file, default_namespace = node_namespace)
    else:
      rospy.logwarn(self.node_name + ": No eligible config file found for " + node_namespace + " in " + self.NEPI_IDX_SENSOR_PARAM_PATH)
    
if __name__ == '__main__':
  node = IDXSensorMgr()            

        
      

 