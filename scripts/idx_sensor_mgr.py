#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import subprocess
import time
import rosparam
import rospy

# Needed for GenICam auto-detect
from harvesters.core import Harvester

class IDXSensorMgr:
  DEFAULT_NODE_NAME = "idx_sensor_mgr"

  NEPI_DEFAULT_CFG_PATH = '/opt/nepi/ros/etc/'
  V4L2_SENSOR_CHECK_INTERVAL_S = 3.0
  GENICAM_SENSOR_CHECK_INTERVAL_S = 3.0

  # Not all "v4l2-ctl --list-devices" reported devices are desirable
  DEFAULT_EXCLUDED_V4L2_DEVICES = ['msm_vidc_vdec'] # RB5 issue work-around for now

  DEFAULT_ZED_V4L2_DEVICES = ['ZED 2',         # Don't treat ZED as V4L2 device - use its own SDK instead
                              'ZED 2i',
                              'ZED-M']         # TODO: Not sure this is how Zed mini is identified by v4l2-ctl... need to test when hardware available ]
  
  DEFAULT_EXCLUDED_GENICAM_DEVICES = []  # None at present

  DEFAULT_GENTL_PRODUCER_USB = '/opt/baumer/gentl_producers/libbgapi2_usb.cti.2.14.1'
  DEFAULT_GENTL_PRODUCER_GIGE = '/opt/baumer/gentl_producers/libbgapi2_gige.cti.2.14.1'

  def __init__(self):
    # Launch the ROS node
    rospy.loginfo("Starting " + self.DEFAULT_NODE_NAME)
    rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    self.node_name = rospy.get_name().split('/')[-1]

    self.sensorList = []
    self.excludedV4L2Devices = rospy.get_param('~excluded_v4l2_devices', self.DEFAULT_EXCLUDED_V4L2_DEVICES)
    self.excludedGenicamDevices = rospy.get_param('~excluded_genicam_devices', self.DEFAULT_EXCLUDED_GENICAM_DEVICES)
    self.zedV4L2Devices = rospy.get_param('~zed_v4l2_devices', self.DEFAULT_ZED_V4L2_DEVICES)

    self.genicam_harvester = Harvester()
    
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_GIGE)
    self.genicam_harvester.add_file(self.DEFAULT_GENTL_PRODUCER_USB)

    rospy.Timer(rospy.Duration(self.V4L2_SENSOR_CHECK_INTERVAL_S), self.detectAndManageV4L2Sensors)
    rospy.Timer(rospy.Duration(self.GENICAM_SENSOR_CHECK_INTERVAL_S), self.detectAndManageGenicamSensors)
    rospy.spin()

  def detectAndManageGenicamSensors(self, _):
    # Make sure our genicam harvesters context is up to date.
    self.genicam_harvester.update()

    # Take note of any genicam nodes currently running. If they are not found
    # in the current genicam harvesters context, we must assume that they have
    # been disconnected and stop the corresponding node(s).
    active_devices = {d["node_namespace"]: False for d in self.sensorList\
                                                 if d["sensor_class"] == "genicam"}

    # Iterate through each device in the current context.
    for device in self.genicam_harvester.device_info_list:
      model = device.model
      sn = device.serial_number
      vendor = device.vendor
      device_is_known = False

      # Look to see if this device has already been launched as a node. If it
      # has, do nothing. If it hasn't, spin up a new node.
      for known_device in self.sensorList:
        if known_device["sensor_class"] != "genicam":
          continue
        try:
          # The call to communicate() will timeout if the node is still running.
          # If the node has exited, we log the corresponding stdout and stderr
          # and allow it to be restarted.
          stdo, stde = known_device["node_subprocess"].communicate(timeout=0.1)
          rospy.logerr(f'{known_device["node_name"]} exited')
          rospy.logerr(f"stdout: {stdo}")
          rospy.logerr(f"stderr: {stde}")
        except subprocess.TimeoutExpired:
          pass
        device_is_known = (device_is_known or (known_device["model"] == device.model and\
                known_device["serial_number"] == device.serial_number))
        if device_is_known:
          active_devices[known_device["node_namespace"]] = True
          break
      if not device_is_known:
        self.startGenicamSensorNode(vendor=vendor, model=model, serial_number=sn)

    # Stop any nodes associated with devices that have disappeared.
    for node_namespace, running in active_devices.items():
      if not running:
        self.stopAndPurgeSensorNode(node_namespace)

  def detectAndManageV4L2Sensors(self, _): # Extra arg since this is a rospy Timer callback
    # First grab the current list of known V4L2 devices
    p = subprocess.Popen(['v4l2-ctl', '--list-devices'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
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
        # Some v4l2-ctl outputs have an additional ':'
        tmp_device_type = tmp_device_type.split(':')[0]
        
        # Honor the exclusion list
        if tmp_device_type in self.excludedV4L2Devices:
          tmp_device_type = None
          continue

      elif (tmp_device_type != None) and (line.startswith('/dev/video')):
        tmp_device_path = line

        # Make sure this is a legitimate Video Capture device, not a Metadata Capture device, etc.
        is_video_cap_device = False
        p = subprocess.Popen(['v4l2-ctl', '-d', tmp_device_path, '--all'],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        stdout,_ = p.communicate()
        all_out = stdout.splitlines()
        in_device_caps = False
        for all_line in all_out:
          if ('Device Caps' in all_line):
            in_device_caps = True
          elif in_device_caps:
            if ('Video Capture' in all_line):
              is_video_cap_device = True
            break
          elif ':' in all_line:
            in_device_caps = False
        
        if not is_video_cap_device:
          continue

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

  def startGenicamSensorNode(self, vendor, model, serial_number):
    # TODO: fair to assume uniqueness of device serial numbers?
    vendor_ros = vendor.split()[0].replace('-', '_').lower() # Some vendors have really long strings, so just use the part to the first space
    model_ros = model.replace('-', '_').replace(' ', '_').lower()
    serial_number_ros = serial_number.replace('-', '_').replace(' ', '_').lower()
    # TODO: Validate that the resulting rootname is a legal ROS identifier
    root_name = f'{vendor_ros}_{model_ros}'
    unique_root_name = root_name + '_' + serial_number
    node_needs_serial_number = False
    for sensor in self.sensorList:
      if sensor['device_type'] == model:
        node_needs_serial_number = True
        break
    sensor_node_name = root_name if not node_needs_serial_number else unique_root_name
    sensor_node_namespace = rospy.get_namespace() + sensor_node_name
    rospy.loginfo(f"{self.node_name}: Initiating new Genicam node {sensor_node_namespace}")

    self.checkLoadConfigFile(node_name=sensor_node_name)

    rospy.loginfo(f"{self.node_name}: Starting {sensor_node_name} via rosrun")

    # NOTE: have to make serial_number look like a string by prefixing with "sn", otherwise ROS
    #       treats it as an int param and it causes an overflow. Better way to handle this?
    sensor_node_run_cmd = ["rosrun", "nepi_edge_sdk_genicam", "genicam_camera_node.py",
        f"__name:={sensor_node_name}", f"_model:={model}", f"_serial_number:=sn{serial_number}"]
    p = subprocess.Popen(sensor_node_run_cmd)
    if p.poll() is not None:
      rospy.logerr(f'Failed to start {sensor_node_name} via {" ".join(x for x in sensor_node_run_cmd)} (rc = {p.returncode})')
    else:
      self.sensorList.append({"sensor_class": "genicam",
                              "model": model,
                              "serial_number": serial_number,
                              "device_type": model,
                              "node_name": sensor_node_name,
                              "node_namespace": sensor_node_namespace,
                              "node_subprocess": p})

  def startV4L2SensorNode(self, type, path):
    # First, get a unique name
    if type not in self.zedV4L2Devices:
      root_name = type.replace(' ','_').lower()
    else:
      root_name = type.replace('-','').replace(' ','').lower() # e.g., ZED 2 ==> zed2 (important that it matches an existing Zed ROS Wrapper launch file)

    same_type_count = 0
    for sensor in self.sensorList:
      if sensor['device_type'] == type:
        same_type_count += 1

    sensor_node_name = root_name
    if same_type_count > 0:
      sensor_node_name += '_' + str(same_type_count)

    sensor_node_namespace = rospy.get_namespace() + sensor_node_name
    rospy.loginfo(self.node_name + ": Initiating new V4L2 node " + sensor_node_namespace)

    self.checkLoadConfigFile(node_name = sensor_node_name)

    # Now start the node via rosrun
    # rosrun nepi_edge_sdk_v4l2 v4l2_camera_node.py __name:=usb_cam_1 _device_path:=/dev/video0
    if type not in self.zedV4L2Devices:
      sensor_node_run_cmd = ['rosrun', 'nepi_edge_sdk_v4l2', 'v4l2_camera_node.py', '__name:=' + sensor_node_name, '_device_path:='+path]
    else:
      sensor_node_run_cmd = ['rosrun', 'nepi_edge_sdk_v4l2', 'zed_camera_node.py', '__name:=' + sensor_node_name, '_zed_type:=' + root_name]

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
  node = IDXSensorMgr()            

        
      

 
