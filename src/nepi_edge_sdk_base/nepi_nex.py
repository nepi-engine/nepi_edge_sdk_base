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
# 1) NEPI NEX Driver Settings utility functions

import rospy
  
#########################
### NEX Driver Settings Helper Functions


SETTING_TYPES = ["Discrete","String","Bool","Int","Float"]

NONE_SETTINGS = [["None","None","None"]]

TEST_CAP_SETTINGS = [["Discrete","TestDiscrete","Option_1","Option_2","Option_3","Option_4"],
  			["String","TestString"],
  			["Bool","TestBool"],
  			["Int","TestInt","0","100"],
  			["Float","TestFloat"]]

TEST_SETTINGS = [["Discrete","TestDiscrete","Option_1"],
  			["String","TestString","InitString"],
  			["Bool","TestBool","True"],
  			["Int","TestInt","5"],
  			["Float","TestFloat","3.14"]] 

TEST_SETTINGS_UPDATE = [["Discrete","TestDiscrete","Option_3"],
  			["String","TestString","NewString"],
  			["Bool","TestBool","False"],
  			["Int","TestInt","500"],
  			["Float","TestFloat","9.81"]] 

def TEST_UPDATE_FUNCTION_SUCCESS(setting):
  s_str = get_setting_as_str(setting)
  rospy.loginfo("Setting update success: " + s_str)
  return True

def TEST_UPDATE_FUNCTION_FAIL(setting):
  s_str = get_setting_as_str(setting)
  rospy.loginfo("Setting update failed: " + s_str)
  str(2+[2])
  return False

def TEST_UPDATE_FUNCTION_EXCEPTION(setting):
  s_str = get_setting_as_str(setting)
  rospy.loginfo("Setting update will cauase exception: " + s_str)
  str(2+[2])
  return True


def parse_settings_msg_data(msg_data):
  msg_string_list = eval(msg_data)
  setting = []
  settings = []
  if len(msg_string_list) >= 2:
    for string in msg_string_list:
      if string in SETTING_TYPES:
        if setting != []:
          settings.append(setting)
          setting = []
        setting.append(string)
      else:
        setting.append(string)
    if setting != []:
      settings.append(setting) # Update last
  return(settings)

def create_new_settings():
  return []


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


def get_name_str_from_setting(s_name,setting):
  s_name = None
  if len(setting)>2:
    s_name = setting[1]
  return s_name

def get_type_str_from_setting(s_name,setting):
  s_type = None
  if len(setting)>2:
    s_type = setting[0]
  return s_type

def get_value_str_from_setting(s_name,setting):
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
  status = None
  data = None
  if len(setting) == 3:
    if setting[0] != None and setting[2] != None:
      s_name = setting[1]
      s_type = setting[0]
      s_value = setting[2]
      try:
        if s_type == "Bool":
          data = bool(s_value)
        elif s_type == "Int":
          data = int(s_value)
        elif s_type == "Float":
          data = float(s_value)
        elif s_type == "String":
          data = str(s_value)
        elif s_type == "Discrete":
          data = str(s_value)
      except Exception as e:
        rospy.loginfo("Setting conversion failed for setting " + s_str + "with exception" + str(e) )
  return value


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
      c_type = cap_setting[0]
      if s_type == c_type:
        if c_type == "Bool" and isinstance(eval(s_value),bool) :
          valid = True
        if c_type == "Int" and isinstance(eval(s_value),int) :
          valid = True
        elif c_type == "Float" and isinstance(eval(s_value),float) :
          valid = True
        elif c_type == "String" and isinstance(s_value,str):
          valid = True      
        elif c_type == "Discrete" and isinstance(s_value,str):  
          options = cap_setting[2:]
          if s_value in options:
            valid = True
  return valid

def update_setting_in_settings(setting_to_update,settings):
  s_name = setting_to_update[1]
  s_value = setting_to_update[2]
  updated_settings = []
  found_setting = False
  for setting in settings:
    if setting[1] == s_name: # append updated setting
      updated_settings.append(setting_to_update)
      found_setting = True
    else: # append original setting
      updated_settings.append(setting)
  if found_setting is False: # append new setting
      updated_settings.append(update_setting)
  return updated_settings

def remove_setting_from_settings(setting_to_remove,settings):
  updated_settings = []
  for setting in settings:
    if setting_to_remove[0] != setting[0]:
      updated_settings.append(settings)
  return updated_settings
	

def try_to_update_setting(setting_to_update,settings,cap_settings,update_settings_function):
  s_str = get_setting_as_str(setting_to_update)
  updated_settings = settings
  success = False
  if update_settings_function is not None:
    if check_valid_setting(setting_to_update,cap_settings):
      try:
        if update_settings_function(setting_to_update):
           updated_settings = update_setting_in_settings(setting_to_update,settings)
        else:
          rospy.loginfo("Failed to update setting: " + s_str + " Update function return failure message")
      except Exception as e:
        rospy.loginfo("Failed to update setting: " + s_str + " with exception: " + str(e) )
    else:
      rospy.loginfo("Failed to update setting: " + s_str + " Invalid setting name or value")
  else:
    rospy.loginfo("Failed to update setting: " + s_str + " No update function provided")
  return updated_settings, success


def filter_settings_by_type(settings,type_str):
  settings_of_type = []
  for setting in settings:
    if len(setting) > 1:
      if setting[0] == type_str :
        settings_of_type.append(setting)
  return settings_of_type

def sort_settings_alphabetically(settings,setting_ind_to_sort):
  if len(settings) > 1:
    sorted_settings = sorted(settings, key=lambda x: x[setting_ind_to_sort])
    return sorted_settings
  else:
    return settings
	
  
def create_msg_data_from_settings(settings):
  msg_data = []
  for setting in settings:
    for string in setting:
      msg_data.append(string)
  return str(msg_data)




  
