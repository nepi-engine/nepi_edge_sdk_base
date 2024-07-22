#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI utility script includes
# 1) RBX Initialization Process
# 2) RBX Settings Utility Functions
# 3) RBX Control Utility Functions


import rospy
import rosnode
import time
import sys
from nepi_edge_sdk_base import nepi_ros

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Bool, String, Float32, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome
from nepi_ros_interfaces.msg import RBXInfo, RBXStatus, AxisControls, RBXErrorBounds, RBXGotoErrors, \
    RBXGotoPose, RBXGotoPosition, RBXGotoLocation, SettingUpdate
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, RBXCapabilitiesQuery, \
     RBXCapabilitiesQueryResponse, NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse


# ROS namespace setup

#######################
# RBX initialization process

def rbx_initialize(self, rbx_namespace):
  ## Initialize Class Variables
  rbx_caps = RBXCapabilitiesQueryResponse()
  self.rbx_cap_states = [""]
  self.rbx_cap_modes = [""]
  self.rbx_cap_actions = [""]
  rbx_caps_navpose = NavPoseCapabilitiesQueryResponse()
  self.rbx_settings = None
  self.rbx_info = None
  self.rbx_status= None


  ## Define Class Namespaces
  # NEPI RBX DEVICE NAMESPACE
  rbx_topic=nepi_ros.wait_for_topic(rbx_namespace)
  NEPI_ROBOT_NAMESPACE = rbx_topic.rpartition("rbx")[0]
  NEPI_RBX_NAMESPACE = (NEPI_ROBOT_NAMESPACE + "rbx/")
  rospy.loginfo("Found rbx namespace: " + NEPI_RBX_NAMESPACE)
  rospy.loginfo("Found rbx status topic: " + rbx_topic)
  # NEPI RBX Driver Service Topics
  NEPI_RBX_CAPABILITIES_TOPIC = NEPI_RBX_NAMESPACE + "capabilities_query"
  NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC = NEPI_RBX_NAMESPACE + "navpose_capabilities_query"
  # Get RBX capabilities
  nepi_ros.wait_for_service(NEPI_RBX_CAPABILITIES_TOPIC)
  rbx_caps_service = rospy.ServiceProxy(NEPI_RBX_CAPABILITIES_TOPIC, RBXCapabilitiesQuery)
  time.sleep(1)
  rbx_caps = rbx_caps_service()
  rospy.loginfo(rbx_caps)
  self.rbx_cap_states = eval(rbx_caps.state_options)
  self.rbx_cap_modes = eval(rbx_caps.mode_options)
  self.rbx_cap_actions = eval(rbx_caps.action_options)
  # rospy.loginfo some results
  rospy.loginfo("RBX State Options: ")
  for state in self.rbx_cap_states:
     rospy.loginfo(" - " + state)
  rospy.loginfo("RBX Mode Options: ")
  for mode in self.rbx_cap_modes:
     rospy.loginfo(" - " + mode)
  rospy.loginfo("RBX Action Options: ")
  for action in self.rbx_cap_actions:
     rospy.loginfo(" - " + action)


  # NEPI RBX Driver Control Topics
  NEPI_RBX_SET_STATE_TOPIC = NEPI_RBX_NAMESPACE + "set_state" # Int to Defined Dictionary RBX_STATES
  NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"  # Int to Defined Dictionary RBX_MODES
  NEPI_RBX_SET_CMD_TIMEOUT_TOPIC = NEPI_RBX_NAMESPACE + "set_cmd_timeout" # Int Seconds  - Any command that changes ready state
  NEPI_RBX_SET_HOME_TOPIC = NEPI_RBX_NAMESPACE + "set_home" # GeoPoint
  NEPI_RBX_SET_HOME_CURRENT_TOPIC = NEPI_RBX_NAMESPACE + "set_home_current" # Empty
  NEPI_RBX_SET_STATUS_IMAGE_TOPIC = NEPI_RBX_NAMESPACE + "set_image_topic" # full or partial ROS namespace
  NEPI_RBX_SET_PROCESS_NAME_TOPIC = NEPI_RBX_NAMESPACE + "set_process_name"  # string name of current process

  self.rbx_set_state_pub = rospy.Publisher(NEPI_RBX_SET_STATE_TOPIC, UInt8, queue_size=1)
  self.rbx_set_mode_pub = rospy.Publisher(NEPI_RBX_SET_MODE_TOPIC, UInt8, queue_size=1)
  self.rbx_set_cmd_timeout_pub = rospy.Publisher(NEPI_RBX_SET_CMD_TIMEOUT_TOPIC, UInt32, queue_size=1)
  self.rbx_set_home_pub = rospy.Publisher(NEPI_RBX_SET_HOME_TOPIC, GeoPoint, queue_size=1)
  self.rbx_set_home_current_pub = rospy.Publisher(NEPI_RBX_SET_HOME_CURRENT_TOPIC, Empty, queue_size=1)
  self.rbx_set_image_topic_pub = rospy.Publisher(NEPI_RBX_SET_STATUS_IMAGE_TOPIC, String, queue_size=1)
  self.rbx_set_process_name_pub = rospy.Publisher(NEPI_RBX_SET_PROCESS_NAME_TOPIC, String, queue_size=1)

  NEPI_RBX_GO_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "go_action"  # Int to Defined Dictionary RBX_ACTIONS
  NEPI_RBX_GO_HOME_TOPIC = NEPI_RBX_NAMESPACE + "go_home" # Aborts any active goto processes
  NEPI_RBX_GO_STOP_TOPIC = NEPI_RBX_NAMESPACE + "go_stop" # Aborts any active goto processes
  NEPI_RBX_GOTO_POSE_TOPIC = NEPI_RBX_NAMESPACE + "goto_pose" # Ignored if any active goto processes
  NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position" # Ignored if any active goto processes
  NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location" # Ignored if any active goto processes

  self.rbx_go_action_pub = rospy.Publisher(NEPI_RBX_GO_ACTION_TOPIC, UInt8, queue_size=1)
  self.rbx_go_home_pub = rospy.Publisher(NEPI_RBX_GO_HOME_TOPIC, Empty, queue_size=1)
  self.rbx_go_stop_pub = rospy.Publisher(NEPI_RBX_GO_STOP_TOPIC, Empty, queue_size=1)
  self.rbx_goto_pose_pub = rospy.Publisher(NEPI_RBX_GOTO_POSE_TOPIC, RBXGotoPose, queue_size=1)
  self.rbx_goto_position_pub = rospy.Publisher(NEPI_RBX_GOTO_POSITION_TOPIC,RBXGotoPosition, queue_size=1)
  self.rbx_goto_location_pub = rospy.Publisher(NEPI_RBX_GOTO_LOCATION_TOPIC, RBXGotoLocation, queue_size=1)

  # Fake GPS Controls
  NEPI_RBX_FAKE_GPS_ENABLE_TOPIC = NEPI_RBX_NAMESPACE + "set_fake_gps_enable" 
  NEPI_RBX_FAKE_GPS_RESET_TOPIC = NEPI_RBX_NAMESPACE + "reset_fake_gps" 
  self.rbx_enable_fake_gps_pub = rospy.Publisher(NEPI_RBX_FAKE_GPS_ENABLE_TOPIC, Bool, queue_size=1)
  self.rbx_reset_fake_gps_pub = rospy.Publisher(NEPI_RBX_FAKE_GPS_RESET_TOPIC, Empty, queue_size=1)

  rospy.loginfo("RBX initialize process complete")



def get_capabilities(self,caps_topic):
  nepi_ros.wait_for_service(caps_topic)
  rbx_caps_service = rospy.ServiceProxy(caps_topic, RBXCapabilitiesQuery)
  time.sleep(1)
  rbx_caps = rbx_caps_service()
  return rbx_caps

def get_navpose_capabilities(self,caps_navpose_topic):
  nepi_ros.wait_for_service(caps_navpose_topic)
  rbx_cap_navpose_service = rospy.ServiceProxy(caps_navpose_topic, NavPoseCapabilitiesQuery)
  time.sleep(1)
  rbx_cap_navpose = rbx_cap_navpose_service()
  return rbx_caps_navpose

#######################
# RBX Settings Functions

### Function to set rbx state
def set_rbx_state(self,state_str,timeout_s = 5):
  rospy.loginfo("*******************************")  
  rospy.loginfo("Set State Request Recieved: " + state_str)
  success = False
  new_state_ind = -1
  for ind, state in enumerate(self.rbx_cap_states):
    if state == state_str:
      new_state_ind = ind
  if new_state_ind == -1:
    rospy.loginfo("No matching state found")
  else:
    rospy.loginfo("Setting state to: " + state_str)
    self.rbx_set_state_pub.publish(new_state_ind)
    timeout_timer = 0
    sleep_time_sec = 1
    while self.rbx_info.state != new_state_ind and timeout_timer < timeout_s and not rospy.is_shutdown():
      rospy.loginfo("Waiting for rbx state " + self.rbx_cap_states[new_state_ind] + " to set")
      rospy.loginfo("Current rbx state is " + self.rbx_cap_states[self.rbx_info.state])
      time.sleep(sleep_time_sec)
      timeout_timer = timeout_timer + sleep_time_sec
    if self.rbx_info.state == new_state_ind:
      success = True
  rospy.loginfo("Current rbx state is " + self.rbx_cap_states[self.rbx_info.state])
  time.sleep(2)
  return success
  
### Function to set rbx mode
def set_rbx_mode(self,mode_str, timeout_s = 5):
  rospy.loginfo("*******************************")  
  rospy.loginfo("Set Mode Request Recieved: " + mode_str)
  success = False
  new_mode_ind = -1
  for ind, mode in enumerate(self.rbx_cap_modes):
    if mode == mode_str:
      new_mode_ind = ind
  if new_mode_ind == -1:
    rospy.loginfo("No matching mode found")
  else:
    rospy.loginfo("Setting mode to: " + mode_str)
    self.rbx_set_mode_pub.publish(new_mode_ind)
    timeout_timer = 0
    sleep_time_sec = 1
    while self.rbx_info.mode != new_mode_ind and timeout_timer < timeout_s and not rospy.is_shutdown():
      rospy.loginfo("Waiting for rbx mode " + self.rbx_cap_modes[new_mode_ind] + " to set")
      rospy.loginfo("Current rbx mode is " + self.rbx_cap_modes[self.rbx_info.mode])
      time.sleep(sleep_time_sec)
      timeout_timer = timeout_timer + sleep_time_sec
    if self.rbx_info.mode == new_mode_ind:
      success = True
  rospy.loginfo("Current rbx mode is " + self.rbx_cap_modes[self.rbx_info.mode])
  time.sleep(1)
  return success

### Function to set home current
def set_rbx_set_home_current(self):
  rospy.loginfo("*******************************")  
  rospy.loginfo("Set Home Current Request Recieved: ")
  success = False
  self.rbx_set_home_current_pub.publish(Empty())
  success = True
  return success

### Function to set image topic name
def set_rbx_image_topic(self,image_topic):
  rospy.loginfo("*******************************")  
  rospy.loginfo("Set Image Topic Request Recieved: ")
  rospy.loginfo(image_topic)
  success = False
  self.rbx_set_image_topic_pub.publish(image_topic)
  success = True
  return success

### Function to set image topic name
def set_rbx_process_name(self,process_name):
  rospy.loginfo("*******************************")  
  rospy.loginfo("Set Process Name Request Recieved: ")
  rospy.loginfo(process_name)
  success = False
  self.rbx_set_process_name_pub.publish(process_name)
  success = True
  return success

### Function to set home current
def set_rbx_set_home_current(self):
  rospy.loginfo("*******************************")  
  rospy.loginfo("Set Home Current Request Recieved: ")
  success = False
  self.rbx_set_home_current_pub.publish(Empty())
  success = True
  return success

#######################
# RBX Control Functions

### Function to send rbx action control
def go_rbx_action(self,action_str,timeout_s = 10):
  rospy.loginfo("*******************************")  
  rospy.loginfo("Goto Action Request Recieved: " + action_str)
  success = False
  action_ind = -1
  for ind, action in enumerate(self.rbx_cap_actions):
    if action == action_str:
      action_ind = ind
  if action_ind == -1:
    rospy.loginfo("No matching action found")
  else:
    rospy.loginfo("Waiting for ready state for takeoff")
    ready = wait_for_rbx_status_ready(self,timeout_s)
    if ready == True:
      rospy.loginfo("Sending takeoff command")
      self.rbx_go_action_pub.publish(action_ind)
      ready = wait_for_rbx_status_busy(self,timeout_s)
      if ready == False:
        ready = wait_for_rbx_status_ready(self,timeout_s)
    success = self.rbx_status.cmd_success
  return success

### Function to send rbx home control
def go_rbx_home(self,timeout_s = 10):
  rospy.loginfo("*******************************")  
  rospy.loginfo("Go Home Request Recieved: ")
  success = False
  ready = wait_for_rbx_status_ready(self,timeout_s)
  if ready == True:
    self.rbx_go_home_pub.publish(Empty())
    ready = wait_for_rbx_status_busy(self,timeout_s)
    if ready == False:
      ready = wait_for_rbx_status_ready(self,timeout_s)
    success = self.rbx_status.cmd_success
  return success

### Function to send rbx home control
def go_rbx_stop(self):
  rospy.loginfo("*******************************")  
  rospy.loginfo("Go Stop Request Recieved: ")
  self.rbx_go_stop_pub.publish(Empty())
  return success
  

### Function to call goto Attititude NED control
def goto_rbx_pose(self,goto_data,timeout_s = 10):
  # Send goto Attitude Command
  if len(goto_data) == 3:
    ready = wait_for_rbx_status_ready(self,timeout_s)
    if ready == True:
      rospy.loginfo("Starting goto Attitude NED Process")
      goto_msg = RBXGotoPose()
      goto_msg.roll_deg = goto_data[0]
      goto_msg.pitch_deg = goto_data[1]
      goto_msg.yaw_deg = goto_data[2]
      self.rbx_goto_pose_pub.publish(goto_msg)
      ready = wait_for_rbx_status_busy(self,timeout_s)
      if ready == False:
        ready = wait_for_rbx_status_ready(self,timeout_s)
    return self.rbx_status.cmd_success
  else:
    return False


### Function to call goto Location Global control
def goto_rbx_location(self,goto_data,timeout_s = 10):
  # Send goto Location Command
  if len(goto_data) == 4:
    ready = wait_for_rbx_status_ready(self,timeout_s)
    if ready == True:
      rospy.loginfo("Starting goto Location Global Process")
      goto_msg = RBXGotoLocation()
      goto_msg.lat = goto_data[0]
      goto_msg.long = goto_data[1]
      goto_msg.altitude_meters= goto_data[2]
      goto_msg.yaw_deg = goto_data[3]
      self.rbx_goto_location_pub.publish(goto_msg)
      ready = wait_for_rbx_status_busy(self,timeout_s)
      if ready == False:
        ready = wait_for_rbx_status_ready(self,timeout_s)
    return self.rbx_status.cmd_success
  else:
    return False

### Function to call goto Position Body control
def goto_rbx_position(self,goto_data,timeout_s = 10):
  # Send goto Position Command
  if len(goto_data) == 4:
    ready = wait_for_rbx_status_ready(self,timeout_s)
    if ready == True:
      rospy.loginfo("Starting goto Position Body Process")
      goto_msg = RBXGotoPosition()
      goto_msg.x_meters = goto_data[0]
      goto_msg.y_meters = goto_data[1]
      goto_msg.z_meters= goto_data[2]
      goto_msg.yaw_deg = goto_data[3]
      self.rbx_goto_position_pub.publish(goto_msg)
      ready = wait_for_rbx_status_busy(self,timeout_s)
      if ready == False:
        ready = wait_for_rbx_status_ready(self,timeout_s)
    return self.rbx_status.cmd_success
  else:
    return False


  
### Function to wait for goto control process to complete
def wait_for_rbx_status_ready(self,timeout_s = 10):
  rospy.loginfo("Waiting for status ready = True")
  count_goal = 3 # fix for strange ready glitch
  counter = 0
  timeout_timer = 0
  sleep_time_sec = 0.1
  while (counter < count_goal) and timeout_timer < timeout_s and (not rospy.is_shutdown()):
    if self.rbx_status.ready is True:
      counter = counter + 1
      #rospy.loginfo("Status ready counter updated to: " + str(self.rbx_status.ready))
    else:
      counter = 0
      #rospy.loginfo("Status ready counter reset")
    time.sleep(sleep_time_sec)
    timeout_timer += sleep_time_sec
  if timeout_timer > timeout_s:
    rospy.loginfo("Aborted Wait for Ready due to timeout" + str(timeout_timer))
  else:
    rospy.loginfo("Got status ready True")
  return self.rbx_status.ready 

### Function to wait for goto control process to complete
def wait_for_rbx_status_busy(self,timeout_s = 10):
  rospy.loginfo("Waiting for status ready = False")
  count_goal = 3 # fix for strange ready glitch
  counter = 0
  timeout_timer = 0
  sleep_time_sec = 0.1
  while (counter < count_goal) and timeout_timer < timeout_s and (not rospy.is_shutdown()):
    if self.rbx_status.ready is False:
      counter = counter + 1
      #rospy.loginfo("Status busy counter updated to: " + str(self.rbx_status.ready))
    else:
      counter = 0
      #rospy.loginfo("Status busy counter reset")
    time.sleep(sleep_time_sec)
    timeout_timer += sleep_time_sec
  if timeout_timer > timeout_s:
    rospy.loginfo("Aborted Wait for Busy due to timeout: " + str(timeout_timer))
  else:
    rospy.loginfo("Got status ready True")
  return self.rbx_status.ready == False




