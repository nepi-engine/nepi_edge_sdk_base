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
# 1) ROS Node utility functions
# 2) ROS Topic utility functions
# 3) ROS Service utility functions
# 4) ROS Param utility functions
# 5) ROS publisher, subscriber, and service
# 6) ROS Time utility functions
# 7) Misc helper functions

  
import os
import sys
import importlib
import shutil
import rospy
import rosnode
import rostopic
import rosservice
import rosparam
import time


from datetime import datetime
from std_msgs.msg import Empty, Float32
from std_srvs.srv import Empty, EmptyRequest, Trigger
from nepi_ros_interfaces.srv import GetScriptsQuery,GetRunningScriptsQuery ,LaunchScript, StopScript
from nepi_ros_interfaces.msg import  Setting


#######################
### Node Utility Functions

def init_node(name):
  rospy.init_node(name)
  
def get_base_namespace():
  nepi_node=find_node('nepi')
  nepi_names = nepi_node.split('/')
  base_namespace = ('/' + nepi_names[1] + '/' + nepi_names[2] + '/')
  return base_namespace

def get_node_namespace():
  return rospy.get_name()
  
def get_node_name():
  return get_node_namespace().split('/')[-1]



# Function to get list of active topics
def get_node_list():
  node_list=rosnode.get_node_names()
  return node_list

### Function to find a node
def find_node(node_name):
  node = ""
  node_list=get_node_list()
  #rospy.loginfo(node_list)
  for node_entry in node_list:
    #rospy.loginfo(node_entry[0])
    if node_entry.find(node_name) != -1:
      node = node_entry
      break
  return node

### Function to check for a node 
def check_for_node(node_name):
  node_exists = True
  node=find_node(node_name)
  if node == "":
    node_exists = False
  return node_exists

### Function to wait for a node
def wait_for_node(node_name):
  rospy.loginfo("NEPI_ROS: Waiting for node with name: " + node_name)
  node = ""
  while node == "" and not rospy.is_shutdown():
    node=find_node(node_name)
    time.sleep(.1)
  rospy.loginfo("NEPI_ROS: Found node: " + node)
  return node

def spin():
  rospy.spin()
  

#######################
### Topic Utility Functions

# Function to get list of active topics
def get_topic_list():
  pubs, subs =rostopic.get_topic_list()
  topic_list = pubs + subs
  return topic_list

# Function to find a topic
def find_topic(topic_name):
  topic = ""
  topic_list=get_topic_list()
  for topic_entry in topic_list:
    topic_str = topic_entry[0]
    if isinstance(topic_str,str):
      if topic_str.find(topic_name) != -1 and topic_str.find(topic_name+"_") == -1:

        topic = topic_str
        break
  return topic

### Function to check for a topic 
def check_for_topic(topic_name):
  topic_exists = True
  topic=find_topic(topic_name)
  if topic == "":
    topic_exists = False
  return topic_exists

# Function to wait for a topic
def wait_for_topic(topic_name):
  rospy.loginfo("NEPI_ROS: Waiting for topic with name: " + topic_name)
  topic = ""
  while topic == "" and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.1)
  rospy.loginfo("NEPI_ROS: Found topic: " + topic)
  return topic

#######################
### Service Utility Functions

# Function to get list of active topics
def get_service_list():
  service = ""
  service_list=rosservice.get_service_list()
  return service_list

# Function to get list of active services
def get_published_service_list(search_namespace='/'):
  service = ""
  service_list=rospy.get_published_services(namespace=search_namespace)
  return service_list

# Function to find a service
def find_service(service_name):
  service = ""
  service_list=get_service_list()
  #rospy.loginfo(service_list)
  for service_entry in service_list:
    #rospy.loginfo(service_entry[0])
    if service_entry.find(service_name) != -1 and service_entry.find(service_name+"_") == -1:
      service = service_entry
      break
  return service

### Function to check for a service 
def check_for_service(service_name):
  service_exists = True
  service=find_service(service_name)
  if service == "":
    service_exists = False
  return service_exists

# Function to wait for a service
def wait_for_service(service_name):
  rospy.loginfo("NEPI_ROS: Waiting for servcie name: " + service_name)
  service = ""
  while service == "" and not rospy.is_shutdown():
    service=find_service(service_name)
    time.sleep(.1)
  rospy.loginfo("NEPI_ROS: Found service: " + service)
  return service


#########################
### Param Utility Functions

def has_param(self,param_namespace):
  return rospy.has_param(param_namespace)

def get_param(self,param_namespace,fallback_param = "None"):
  if fallback_param == "None":
    param = rospy.get_param(param_namespace)
  else:
    param = rospy.get_param(param_namespace,fallback_param)
  return param

def set_param(self,param_namespace,param):
  rospy.set_param(param_namespace,param)

def load_params_from_file(file_path, params_namespace = None):
    if params_namespace is not None:
      if params_namespace[-1] != "/":
        params_namespace += "/"
    else:
      params_namespace = ""
    try:
        params = rosparam.load_file(file_path)
        for param, value in params[0].items():
            param_namesapce = params_namespace + param
            rospy.set_param(param_namesapce, value)
        rospy.loginfo("Parameters loaded successfully from {}".format(file_path))
    except rosparam.RosParamException as e:
        rospy.logerr("Error loading parameters: {}".format(e))


#########################
### Publisher, Subscriber, and Service Utility Functions

def start_timer_process(duration, callback_function, oneshot = False):
  rospy.Timer(duration, callback_function, oneshot)

'''
def getPublisher(namespace, msg_type, queue_size=1):
  return rospy.Publisher(namespace, msg_type, queue_size)

def startSubscriber(namespace, msg_type, callback_function, queue_size=1):
  return rospy.Subscriber(namespace, msg_type, callback_function, queue_size)
'''

#########################
### Time Helper Functions

def duration(time_s):
  return rospy.Duration(time_s)

def time_from_timestamp(timestamp):
  return rospy.Time.from_sec(timestamp)
  
def time_now():
  return rospy.Time.now()


# Sleep process that breaks sleep into smaller times for better shutdown
def sleep(sleep_sec,sleep_steps = None):
  if sleep_steps is not None:
    delay_timer = 0
    delay_sec = sleep_sec/sleep_steps
    while delay_timer < sleep_sec and not rospy.is_shutdown():
      rospy.sleep(delay_sec)
      delay_timer = delay_timer + delay_sec
  else:
    rospy.sleep(sleep_sec)
  return True

def get_datetime_str_now():
  date_str=datetime.utcnow().strftime('%Y-%m-%d')
  time_str=datetime.utcnow().strftime('%H%M%S')
  ms_str =datetime.utcnow().strftime('%f')[:-3]
  dt_str = (date_str + "T" + time_str + "." + ms_str)
  return dt_str

def get_datetime_str_from_stamp(ros_stamp_msg):
  tm=time.gmtime(ros_stamp_msg.secs)
  year_str = tm_2_str(tm.tm_year)
  mon_str = tm_2_str(tm.tm_mon)
  day_str = tm_2_str(tm.tm_mday)
  hr_str = tm_2_str(tm.tm_hour)
  min_str = tm_2_str(tm.tm_min)
  sec_str = tm_2_str(tm.tm_sec)
  date_str=(year_str + '-' + mon_str + '-' + day_str)
  time_str = (hr_str + min_str + sec_str)
  ms_str= str(ros_stamp_msg.nsecs)[0:3]
  dt_str = (date_str + "T" + time_str + "." + ms_str)
  return dt_str

def tm_2_str(tm_val):
  tm_str = str(tm_val)
  if len(tm_str) == 1:
    tm_str = ('0'+ tm_str)
  return tm_str


#########################
### Misc Helper Functions

def is_shutdown():
  return rospy.is_shutdown()

def signal_shutdown(msg):
  rospy.signal_shutdown(msg)

def on_shutdown(shutdown_fuction):
  rospy.on_shutdown(shutdown_fuction)

def parse_string_list_msg_data(msg_data):
  str_list = []
  if msg_data[0] == "[" and msg_data[-1] == "]" :
    str_list = eval(msg_data)
  return(str_list)




# Function for checking if val in list
def val_in_list(val2check,list2check):
  in_list = False
  if len(list2check) > 0:
    for list_val in list2check:
      #rospy.loginfo(str(val2check) + ' , ' + str(list_val))
      #rospy.loginfo(val2check == list_val)
      if val2check == list_val:
        in_list = True
  return in_list

def get_file_list(search_path,ext_str="png"):
  ind = 0
  file_list = []
  for f in os.listdir(search_path):
    if f.endswith(ext_str):
      #rospy.loginfo('Found image file')
      ind = ind + 1
      file = (search_path + '/' + f)
      file_list.append(file)
  return file_list,ind

def check_make_folder(pathname):
  if not os.path.exists(pathname):
    try:
      os.makedirs(pathname)
      rospy.loginfo("NEPI_ROS: Made folder: " + pathname)
    except rospy.ServiceException as e:
      rospy.loginfo("NEPI_ROS: Failed to make folder: " + pathname + " with exeption" + str(e))
  return os.path.exists(pathname)

def copy_files_from_folder(src_path,dest_path):
  success = True
  files_copied = []
  files_not_copied = []
  if os.path.exists(src_path):
    dest_folders = dest_path.split('/')
    dest_folders.remove('')
    check_folder = ""
    for folder in dest_folders:
      check_folder = check_folder + "/" + folder
      check_make_folder(check_folder)
    if os.path.exists(dest_path):
        files = os.listdir(src_path)
        for file in files:
          srcFile = src_path + "/" + file
          destFile = dest_path + "/" + file
          if not os.path.exists(destFile):
            try:
              shutil.copyfile(srcFile, destFile)
            # If source and destination are same
            except shutil.SameFileError:
                rospy.loginfo("NEPI_ROS: Source and destination represents the same file: " + destFile)
            
            # If destination is a directory.
            except IsADirectoryError:
                rospy.loginfo("NEPI_ROS: Destination is a directory: " + destFile)
            
            # If there is any permission issue
            except PermissionError:
                rospy.loginfo("NEPI_ROS: Permission denied for file copy: " + destFile)
            
            # For other errors
            except:
                rospy.loginfo("NEPI_ROS: Error occurred while copying file: " + destFile)
            if not os.path.exists(destFile):
              files_not_copied.append(file)
              success = False
            else: 
              files_copied.append(file)
              #rospy.loginfo("NEPI_ROS: Copied file: " + file)
    else:
      rospy.loginfo("NEPI_ROS: Did not find and can't make destination folder: " + dest_path)
      success = False
  else:
    rospy.loginfo("NEPI_ROS: Did not find source folder: " + src_path)
    success = False
  return success, files_copied, files_not_copied



 

  
