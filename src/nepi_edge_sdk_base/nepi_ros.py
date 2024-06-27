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
# 3) NEPI ROS Script utility functions
# 4) NEPI Settings utility functions
# 5) Misc helper functions

  
import os
import rospy
import rosnode
import rostopic
import rosservice
import time
import sys

from datetime import datetime
from std_msgs.msg import Empty, Float32
from std_srvs.srv import Empty, EmptyRequest, Trigger
from nepi_ros_interfaces.srv import GetScriptsQuery,GetRunningScriptsQuery ,LaunchScript, StopScript

#######################
### Node Utility Functions

# Function to get list of active topics
def get_node_list():
  node = ""
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
  rospy.loginfo("Waiting for node with name: " + node_name)
  node = ""
  while node == "" and not rospy.is_shutdown():
    node=find_node(node_name)
    time.sleep(.1)
  rospy.loginfo("Found node: " + node)
  return node

def get_base_namespace():
  nepi_node=find_node('nepi')
  nepi_names = nepi_node.split('/')
  base_namespace = ('/' + nepi_names[1] + '/' + nepi_names[2] + '/')
  return base_namespace
  

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
  rospy.loginfo("Waiting for topic with name: " + topic_name)
  topic = ""
  while topic == "" and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.1)
  rospy.loginfo("Found topic: " + topic)
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
  rospy.loginfo("Waiting for servcie name: " + service_name)
  service = ""
  while service == "" and not rospy.is_shutdown():
    service=find_service(service_name)
    time.sleep(.1)
  rospy.loginfo("Found service: " + service)
  return service

#######################
# Script Utility Functions

def startup_script_initialize(self,NEPI_BASE_NAMESPACE):
  ## Initialize Class Variables
  self.scripts_installed_at_start = None
  self.scripts_running_at_start = None
  ## Define Class Namespaces
  AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME = NEPI_BASE_NAMESPACE + "get_scripts"
  AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME = NEPI_BASE_NAMESPACE + "get_running_scripts"
  AUTO_LAUNCH_SCRIPT_SERVICE_NAME = NEPI_BASE_NAMESPACE + "launch_script"
  AUTO_STOP_SCRIPT_SERVICE_NAME = NEPI_BASE_NAMESPACE + "stop_script"
  ## Create Class Service Calls
  self.get_installed_scripts_service = rospy.ServiceProxy(AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME, GetScriptsQuery )
  self.get_running_scripts_service = rospy.ServiceProxy(AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME, GetRunningScriptsQuery )
  self.launch_script_service = rospy.ServiceProxy(AUTO_LAUNCH_SCRIPT_SERVICE_NAME, LaunchScript)
  self.stop_script_service = rospy.ServiceProxy(AUTO_STOP_SCRIPT_SERVICE_NAME, StopScript)
  ## Create Class Publishers
  ## Start Class Subscribers
  ## Start Node Processes
  rospy.loginfo("")
  rospy.loginfo("***********************")
  rospy.loginfo("Starting Initialization")
  ### Get list of installed scripts
  rospy.loginfo("Getting list of installed scripts")
  rospy.loginfo(["Calling service name: " + AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME])
  while self.scripts_installed_at_start == None and not rospy.is_shutdown():
      self.scripts_installed_at_start = get_installed_scripts(self.get_installed_scripts_service)
      if self.scripts_installed_at_start == None:
        rospy.loginfo("Service call failed, waiting 1 second then retrying")
        time.sleep(1)
  #rospy.loginfo("Scripts installed at start:")
  #rospy.loginfo(self.scripts_installed_at_start)
  ### Get list of running scripts
  rospy.loginfo("")
  rospy.loginfo("Getting list of running scripts at start")
  rospy.loginfo(["Calling service name: " + AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME])
  while self.scripts_running_at_start == None and not rospy.is_shutdown():
      self.scripts_running_at_start = get_running_scripts(self.get_running_scripts_service)
      if self.scripts_running_at_start == None:
        rospy.loginfo("Service call failed, waiting 1 second then retrying")
        time.sleep(1)
  #rospy.loginfo("Scripts running at start:")
  #rospy.loginfo(self.scripts_running_at_start)



# Function to get list of installed scripts
def get_installed_scripts(get_installed_scripts_service):
  installed_scripts = None
  try:
    response = get_installed_scripts_service()
    installed_scripts = response.scripts
    #rospy.loginfo("Installed Scripts = " + str(installed_scripts))
  except Exception as e:
    rospy.loginfo("Get installed scripts service call failed: " + str(e))
  return installed_scripts

# Function to get list of running scripts
def get_running_scripts(get_running_scripts_service):
  running_scripts = None
  try:
    response = get_running_scripts_service()
    running_scripts = response.running_scripts
    #rospy.loginfo("Running Scripts = " + str(running_scripts))
  except Exception as e:
    rospy.loginfo("Get running scripts service call failed: " + str(e))
  return running_scripts


# Function to launch a script
def launch_script(script2launch,launch_script_service):
  launch_success=False
  try:
    success = launch_script_service(script=script2launch)
    rospy.loginfo("Launched script: " + str(success))
    launch_success=True
  except Exception as e:
    rospy.loginfo("Launch script service call failed: " + str(e))
  return launch_success

### Function to stop script
def stop_script(script2stop,stop_script_service):
  stop_success=False
  try:
    success = stop_script_service(script=script2stop)
    rospy.loginfo("Stopped script: " + str(success))
    stop_success=True
  except Exception as e:
    rospy.loginfo("Stop script service call failed: " + str(e))
  return stop_success

# Function to start scripts from list
def launch_scripts(script_list,launch_script_service,get_installed_scripts_service,get_running_scripts_service):
  installed_scripts = get_installed_scripts(get_installed_scripts_service)
  running_scripts = get_running_scripts(get_running_scripts_service)
  if installed_scripts is not None and running_scripts is not None:
    for script2launch in script_list:
      script_installed = val_in_list(script2launch,installed_scripts)
      if script_installed:
        script_running = val_in_list(script2launch,running_scripts)
        if script_running is False:
            rospy.loginfo("")
            rospy.loginfo(["Launching script: " + script2launch])
            script_launch = launch_script(script2launch,launch_script_service)
            if script_launch:
              rospy.loginfo("Script launch call success")
              script_running = False
              while script_running is False and not rospy.is_shutdown():
                running_scripts = get_running_scripts(get_running_scripts_service)
                script_running = val_in_list(script2launch,running_scripts)
                rospy.loginfo("Waiting for script to launch")
                time.sleep(.5) # Sleep before checking again
              rospy.loginfo("Script started successfully")
            else:
               rospy.loginfo("Scipt launch call failed")
        else:
          rospy.loginfo("Script already running, skipping launch process")
      else:
        rospy.loginfo("Script not found, skipping launch process")
  else:
    rospy.loginfo("Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #rospy.loginfo(running_scripts)
          

# Function to stop scripts from list, a
def stop_scripts(script_list,stop_script_service,get_installed_scripts_service,get_running_scripts_service,optional_ignore_script_list=[]):
  installed_scripts = get_installed_scripts(get_installed_scripts_service)
  running_scripts = get_running_scripts(get_running_scripts_service)
  if installed_scripts is not None and running_scripts is not None:
    for script2stop in script_list:
      script_running = val_in_list(script2stop,running_scripts)
      script_ignore = val_in_list(script2stop,optional_ignore_script_list)
      if script_running is True and script_ignore is False:
        script_running = val_in_list(script2stop,running_scripts)
        if script_running is True:
            rospy.loginfo("")
            rospy.loginfo(["Stopping script: " + script2stop])
            script_stop = stop_script(script2stop,stop_script_service)
            if script_stop:
              rospy.loginfo("Script stop call success")
            else:
               rospy.loginfo("Scipt stop call failed")
        else:
          rospy.loginfo("Scipt in ignore list, skipping launch process")
      else:
        rospy.loginfo("Script not found, skipping launch process")
  else:
    rospy.loginfo("Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #rospy.loginfo(running_scripts)


#########################
### Settings Helper Functions


SETTING_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]

NONE_SETTINGS = [["None","None","None"]]

TEST_CAP_SETTINGS = [["Menu","TestMenu","Menu_A:0","Menu_B:1","Menu_C:2","Menu_D:3"],
        ["Discrete","TestDiscrete","Option_1","Option_2","Option_3","Option_4"],
        ["String","TestString"],
        ["Bool","TestBool"],
        ["Int","TestInt_1"],
        ["Int","TestInt_2","0","100"],
        ["Float","TestFloat_1"],
        ["Float","TestFloat_2","-1000","5000"]]

TEST_SETTINGS = [["Menu","TestMenu","Menu_A:0"],
        ["Discrete","TestDiscrete","Option_1"],
        ["String","TestString","InitString"],
        ["Bool","TestBool","True"],
        ["Int","TestInt_1","5"],
        ["Int","TestInt_2","25"],
        ["Float","TestFloat_1","3.14"], 
        ["Float","TestFloat_2","5000"]] 

TEST_SETTINGS_UPDATE = [["Menu","TestMenu","Menu_C:2"],
        ["Discrete","TestDiscrete","Option_3"],
        ["String","TestString","NewString"],
        ["Bool","TestBool","False"],
        ["Int","TestInt_1","500"],
        ["Int","TestInt_2","100"],
        ["Float","TestFloat_1","9.81"], 
        ["Float","TestFloat_2","1000"]] 

def TEST_UPDATE_FUNCTION_SUCCESS(setting):
  s_str = get_setting_as_str(setting)
  rospy.loginfo("Setting update success: " + s_str)
  return True, "Success"

def TEST_UPDATE_FUNCTION_FAIL(setting):
  s_str = get_setting_as_str(setting)
  rospy.loginfo("Setting update failed: " + s_str)
  str(2+[2])
  return False, "Failed just because"

def TEST_UPDATE_FUNCTION_EXCEPTION(setting):
  s_str = get_setting_as_str(setting)
  rospy.loginfo("Setting update will cauase exception: " + s_str)
  str(2+[2])
  return True, "Failed with exception"


def UPDATE_NONE_SETTINGS_FUNCTION():
   return False, "No settings update function available"

def GET_NONE_SETTINGS_FUNCTION():
   return nepi_nex.NONE_SETTINGS
        
def parse_settings_msg_data(msg_data):
  setting = []
  settings = []
  if msg_data[0] == "[" and msg_data[-1] == "]" :
    settings = eval(msg_data)
  return(settings)

def create_new_settings(name_str,type_str,value_str=None,options_str_list=None):
  setting = []
  setting.append(type_str)
  setting.append(name_str)
  if value_str != None:
    setting.append(value_str)
  if options_str_list != None:
    for option_str in option_str_list:
      setting.append(option_str)
  return setting

def create_msg_data_from_settings(settings):
  msg_data = []
  for setting in settings:
      msg_data.append(setting)
  return str(msg_data)


def get_setting_from_settings(s_name,settings):
  for setting in settings:
    if setting[1] == s_name:
      return setting
      break
  return None

def create_setting(s_name,s_type,s_values = [None]):
  setting = None
  if type_s in SETTING_TYPES:
    setting = [s_type,s_name]
    if isinstance(s_value,list):
      if s_value is not None:
        for value in s_values:
          settings.append(value)
      elif isinstance(s_values,str):
          setting.append(s_values)
  return setting

def add_setting_to_settings(setting,settings=[]):
  if len(setting) > 1:
    settings.append(setting)
  return settings

def remove_setting_from_settings(setting,settings):
  updated_settings = []
  for check_setting in settings:
    if setting[0] != check_setting[0]:
      updated_settings.append(settings)
  return updated_settings


def get_setting_as_str(setting):  
  s_str = (setting[0] + ',' + setting[1] + "," + setting[2])
  return s_str


def get_name_str_from_setting(setting):
  s_name = None
  if len(setting)>2:
    s_name = setting[1]
  return s_name

def get_type_str_from_setting(setting):
  s_type = None
  if len(setting)>2:
    s_type = setting[0]
  return s_type

def get_value_str_from_setting(setting):
  s_value = None
  if len(setting)>2:
    s_value = setting[2]
  return s_value

def get_options_from_cap_setting(s_name,setting):
  value = None
  if len(setting)>2:
    value = setting[2:]
  return value

def get_data_from_setting(setting):
  s_str = get_setting_as_str(setting)
  s_name = setting[1]
  s_type = setting[0]
  status = None
  data = None
  if len(setting) == 3:
    if setting[0] != None and setting[2] != None:

      s_value = setting[2]
      try:
        if s_type == "Bool":
          data = (s_value == "True")
        elif s_type == "Int":
          data = int(s_value)
        elif s_type == "Float":
          data = float(s_value)
        elif s_type == "String":
          data = s_value
        elif s_type == "Discrete":
          data = s_value
        elif s_type == "Menu":
          data = int(s_value.split(":")[1])
      except Exception as e:
        rospy.loginfo("Setting conversion failed for setting " + s_str + "with exception" + str(e) )
  return s_name, s_type, data


def compare_setting_in_settings(setting,settings):
  s_name = setting[1]
  s_type = setting[0]
  s_value = setting[2]
  name_match = False
  type_match = False
  value_match = False
  for check_setting in settings:
    if check_setting[1] == s_name:
      name_match = True
      if check_setting[0] == s_type:
        type_match = True
      if check_setting[2] == s_value:
        value_match = True
      return name_match, type_match, value_match
      break
  return name_match, type_match, value_match

def check_valid_setting(setting,cap_settings):
  valid= False
  for cap_setting in cap_settings: # Check for valid option in capabilities string setting
    s_type = setting[0]
    s_name = setting[1]
    s_value = setting[2]
    if cap_setting[1] == s_name:
      ("1")
      c_type = cap_setting[0]
      if len(cap_setting) > 2:
        c_options = cap_setting[2:] 
      else:
        c_options = []
      if s_type == c_type:
        if s_value.find("(") == -1 and s_value.find(")") == -1: # Check that s_value is not a function call before eval call
          if c_type == "Bool" and isinstance(eval(s_value),bool) :
            valid = True
          elif c_type == "String" and isinstance(s_value,str):
            valid = True      
          elif c_type == "Menu" and isinstance(s_value,str):  
            if s_value in c_options:
              valid = True
          elif c_type == "Discrete" and isinstance(s_value,str):  
            if s_value in c_options:
              valid = True
          elif c_type == "Int" and isinstance(eval(s_value),int):
            if len(c_options) == 2:
              val = eval(s_value)
              if c_options[0] != "":
                lower_limit = eval(c_options[0])
              else:
                lower_limit = -float('inf')
              (val >= lower_limit)
              if c_options[1] != "":
                upper_limit = eval(c_options[1])
              else:
                upper_limit = float('inf')
              (val <= upper_limit)
              if val >= lower_limit and val <= upper_limit:
                valid = True
              (valid)
            else:    
              valid = True
          elif c_type == "Float" and (isinstance(eval(s_value),float) or isinstance(eval(s_value),int)):
            if len(c_options) == 2:
              val = eval(s_value)
              if c_options[0] != "":
                lower_limit = eval(c_options[0])
              else:
                lower_limit = -float('inf')
              (val >= lower_limit)
              if c_options[1] != "":
                upper_limit = eval(c_options[1])
              else:
                upper_limit = float('inf')
              (val <= upper_limit)
              if val >= lower_limit and val <= upper_limit:
                valid = True
              (valid)
            else:    
              valid = True
  return valid

def update_setting_in_settings(setting_to_update,settings):
  s_name = setting_to_update[1]
  s_value = setting_to_update[2]
  updated_settings = []
  found_setting = False
  for setting in settings:
    if setting[1] == s_name: 
      updated_settings.append(setting_to_update) # append/replace new
      found_setting = True
    else: 
      updated_settings.append(setting) # append original
  if found_setting is False: # append new setting to original settings
      updated_settings.append(setting_to_update) # append/add new
  return updated_settings

def remove_setting_from_settings(setting_to_remove,settings):
  updated_settings = []
  for setting in settings:
    if setting_to_remove[0] != setting[0]:
      updated_settings.append(settings)
  return updated_settings
	

def try_to_update_setting(setting_to_update,settings,cap_settings,update_settings_function):
  s_str = get_setting_as_str(setting_to_update)
  s_name = get_name_str_from_setting(setting_to_update)
  updated_settings = settings
  success = False
  msg = ""
  if s_name != "None":
    if update_settings_function is not None:
      if check_valid_setting(setting_to_update,cap_settings):
        try:
          [success, msg] = update_settings_function(setting_to_update)
          if success:
            msg = ("Updated setting: " + s_str )
        except Exception as e:
          msg = ("Failed to update setting: " + s_str + " with exception: " + str(e) )
      else:
        msg = ("Failed to update setting: " + s_str + " Invalid setting name or value")
    else:
      msg = ("Failed to update setting: " + s_str + " No update function provided")
  else:
    success = True
  return success, msg


def get_settings_by_type(settings,type_str):
  settings_of_type = []
  for setting in settings:
    if len(setting) > 1:
      if setting[0] == type_str :
        settings_of_type.append(setting)
  return settings_of_type

def sort_settings_alphabetically(settings):
  if len(settings) > 1:
    sorted_settings = sorted(settings, key=lambda x: x[1])
    return sorted_settings
  else:
    return settings

#########################
### Misc Helper Functions

# Sleep process that breaks sleep into smaller times for better shutdown
def sleep(sleep_sec,sleep_steps):
  delay_timer = 0
  delay_sec = sleep_sec/sleep_steps
  while delay_timer < sleep_sec and not rospy.is_shutdown():
    time.sleep(delay_sec)
    delay_timer = delay_timer + delay_sec

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

def get_file_list(image_dir,ext_str="png"):
  path, dirs, files = next(os.walk(image_dir))
  data_size = len(files)
  ind = 0
  file_list = []
  for f in os.listdir(image_dir):
    if f.endswith(ext_str):
      #print('Found image file')
      ind = ind + 1
      file = (image_dir + '/' + f)
      file_list.append(file)
  return file_list,ind
 

  
