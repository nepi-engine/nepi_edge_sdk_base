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


  
#########################
### NEX Driver Settings Helper Functions

def parse_cap_msg_settings_data(cap_msg_data):
  cap_msg_data = eval(cap_msg_data)
  new_setting = False
  last_string = None
  cap_setting = []
  cap_settings = []
  #print("***Working on string setting")
  #print(setting_str_msg)
  if cap_msg_data != []:
    for string in cap_msg_data:
      #print("")
      #print(last_string)
      #print(string)
      if string == "Discrete" or string == "String" or string == "Bool" or string == "Int" or string == "Float":
        if cap_setting != []:
          #print(cap_setting)
          cap_setting =  cap_setting[0:-1] # Remove last string
          cap_settings.append(cap_setting) # Update last 
          #print(cap_settings)
          cap_setting = []
        cap_setting= [last_string,string]
        new_setting = True
      elif cap_setting != []:
        cap_setting.append(string)
      last_string = string
    cap_settings.append(cap_setting) # Update last
  #print(cap_settings) 
  return(cap_settings)

def parse_status_msg_settings_data(status_msg_data):
  status_msg_data = eval(status_msg_data)
  str_ind = 0
  status_setting = []
  status_settings = []
  if status_msg_data != []:
    for string in status_msg_data:
      status_setting.append(string)
      if str_ind < 2:
        str_ind = str_ind + 1
      else:
        status_settings.append(status_setting)
        str_ind = 0
        status_setting = []
  return(status_settings)

def parse_update_settings_msg_data(update_msg_data):
  update_msg_data = eval(update_msg_data)
  str_ind = 0
  update_setting = []
  update_settings = []
  if update_msg_data != []:
    for string in update_msg_data:
      update_setting.append(string)
      if str_ind < 1:
        str_ind = str_ind + 1
      else:
        update_settings.append(update_setting)
        str_ind = 0
        update_setting = []
  return(update_settings)

def get_setting_types():
  types_setting = ["Discrete","String","Bool","Int","Float"]
  return types_setting

def filter_settings_by_type(cap_settings,type_str):
  cap_settings_of_type = []
  for cap_setting in cap_settings:
    if len(cap_setting) > 1:
      if cap_setting[1] == type_str :
        cap_settings_of_type.append(cap_setting)
  return cap_settings_of_type


def get_status_setting_from_status_settings(status_name,status_settings):
  get_status_setting = None
  for status_setting in status_settings:
    if status_setting[0].find(status_name):
      get_status_setting = status_setting
      return get_status_setting
      break
  return None

def get_value_from_status_setting(status_setting):
  status = None
  value = None
  if len(status_setting) == 3:
    if status_setting[1] != None and status_setting[2] != None:
      status = status_setting[0]
      type = status_setting[1]
      value_str = status_setting[2]
      value = None
      if type == "Bool":
        value = eval(value_str)
      elif type == "Int":
        value = int(value_str)
      elif type == "Float":
        value = float(value_str)
      else:
        value = value_str
  return status,value

def get_value_from_status_settings(status_name,status_settings):
  status = None
  value = None
  if status_settings != [[]]:
    for status_setting in status_settings:
     if status_setting[0].find(status_name) != -1:
       [status,value] = get_value_from_status_setting(status_setting)
  return status, value
  
def update_value_in_status_setting(status_name,new_value,status_setting,cap_settings):
  for cap_setting in cap_settings: # Check for valid option in capabilities string setting
    if cap_setting[0].find(status_name) != -1:
      type = cap_setting[1]
      if type == "Bool" and isinstance(eval(new_value),bool) :
        status_setting[2] = str(new_value)
      if type == "Int" and isinstance(eval(new_value),int) :
        status_setting[2] = str(new_value)
      elif type == "Float" and isinstance(eval(new_value),float) :
        status_setting[2] = str(new_value)
      elif type == "String" and isinstance(new_value,str):
        status_setting[2] = new_value      
      elif type == "Discrete" and isinstance(new_value,str):  
        for options in cap_setting[2:]:
          if options.find(new_value):
            status_setting[2] = new_value
  updated_status_setting = status_setting
  return updated_status_setting

def update_value_in_status_settings(status_name,new_value,status_settings,cap_settings):
  updated_status_settings = []
  for status_setting in status_settings:
    if status_setting[0].find(status_name) != -1:
      updated_status_setting = update_value_in_status_setting(status_name,new_value,status_setting,cap_settings)
    else:
      updated_status_setting = status_setting
    updated_status_settings.append(updated_status_setting)
  return updated_status_settings

def sort_settings_alphabetically(input_settings,setting_ind_to_sort):
  if len(input_settings) > 1:
    sorted_settings = sorted(input_settings, key=lambda x: x[setting_ind_to_sort])
    return sorted_settings
  else:
    return input_settings
  
def create_msg_data_from_settings(settings):
  msg_data = []
  for setting in settings:
    for string in setting:
      msg_data.append(string)
  return str(msg_data)




  
