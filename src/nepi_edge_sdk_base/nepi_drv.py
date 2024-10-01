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
# 1) NEPI IDX Driver utility functions
import os
import sys
import zipfile
import getpass
import importlib
import subprocess
import rospy
import rosnode
import warnings
import numpy as np
import time
import usb
import copy
from serial.tools import list_ports

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img



#########################
### Example Requried File Headers
## Exaple Node File Header
'''
PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'NODE'
NODE_DICT = dict(
description = 'Driver package for generic GenICam camera devices',
class_name = 'GenicamCamNode', # Should Match Class Name,
group ='IDX',
group_id = 'GENICAM' ,
driver_pkg_name = 'IDX_GENICAM', # 'Required Driver PKG_NAME or 'None'
discovery_pkg_name = 'IDX_GENICAM' # 'Required Discovery PKG_NAME or 'None'
)
'''
## Exaple Discovery File Header
'''
PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'DISCOVERY'
DISCOVERY_DICT = dict(
  class_name = 'GenicamCamDiscovery',
  process = 'LAUNCH', # 'LAUNCH', 'RUN', or 'CALL'
  method = 'AUTO',  # 'AUTO', 'MANUAL', or 'OTHER' if managed by seperate application
  include_ids = [],  # List of string identifiers for discovery process
  exclude_ids = [], # List of string identifiers for discovery process
  interfaces = ['USB','IP'], # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
  option_1_dict = dict(
    name = 'None',
    options = [], # List of string options. Selected option passed to driver
    default_val = 'None'
  ),
  option_2_dict = dict(
    name = 'None',
    options = [], # List of string options. Selected option passed to driver
    default_val = 'None'
  )
)
'''
## Exaple Driver File Header
'''
PKG_NAME = 'IDX_GENICAM' # Use in display menus
FILE_TYPE = 'DRIVER'
DRIVER_DICT = dict(
class_name = 'GenicamCamDriver'
)
'''



#######################
### Driver Utility Functions

DRIVER_DIR = '/opt/nepi/ros/lib/nepi_drivers'
DRIVER_FILE_TYPES = ['Node','Driver', 'Discovery']
DRIVER_KEYS = ['node','driver','discovery']


def getDriversDict(search_path):
    node_dict = dict()
    discovery_dict = dict()
    driver_dict = dict()

    # Find driver files
    ind = 0
    if os.path.exists(search_path):
        if search_path[-1] == "/":
            search_path = search_path[:-1]
        sys.path.append(search_path)
        #rospy.loginfo("NEPI_DRV: Searching for drivers in path: " + search_path)
        for f in os.listdir(search_path):
          if f.endswith(".py"): 
            module_name = f.split(".")[0]
            #rospy.loginfo("NEPI_DRV: Will try to import module: " + module_name)
            open_success = True
            read_success = True
            warnings.filterwarnings('ignore', '.*unclosed.*', )
            try:
              module = __import__(module_name)
            except Exception as e:
              rospy.logwarn("NEPI_DRV: failed to import module %s with exception %s", f, str(e))
              open_success = False
            if open_success:
                try:
                  module_type = module.FILE_TYPE                
                except:
                  rospy.logwarn("NEPI_DRV: No FILE_TYPE in module: " + f)
                  read_success = False
                if read_success:
                  #rospy.logwarn("NEPI_DRV: " + module_type)

                  if module_type == "NODE":
                    try:
                      pkg_name = module.PKG_NAME
                      node_dict[pkg_name] = module.NODE_DICT
                      node_dict[pkg_name]['pkg_name'] = pkg_name
                      node_dict[pkg_name]['file_name'] = f
                      node_dict[pkg_name]['module_name'] = module_name
                    except Exception as e:
                      rospy.logwarn("NEPI_DRV: Failed to get Node info from module: " + f +" with exception: " + str(e))

                  elif module_type == "DRIVER":
                    try:
                      pkg_name = module.PKG_NAME
                      driver_dict[pkg_name] = module.DRIVER_DICT
                      driver_dict[pkg_name]['pkg_name'] = pkg_name
                      driver_dict[pkg_name]['file_name'] = f
                      driver_dict[pkg_name]['module_name'] = module_name
                    except Exception as e:
                      rospy.logwarn("NEPI_DRV: Failed to get Discovery info from module: " + f +" with exception: " + str(e))

                  elif module_type == "DISCOVERY":
                    try:
                      pkg_name = module.PKG_NAME
                      discovery_dict[pkg_name] = module.DISCOVERY_DICT
                      discovery_dict[pkg_name]['pkg_name'] = pkg_name
                      discovery_dict[pkg_name]['file_name'] = f
                      discovery_dict[pkg_name]['module_name'] = module_name
                      # Initialize some values
                      discovery_dict[pkg_name]['option_1_dict']['set_val'] = module.DISCOVERY_DICT['option_1_dict']['default_val']
                      discovery_dict[pkg_name]['option_2_dict']['set_val'] = module.DISCOVERY_DICT['option_2_dict']['default_val']
                    except Exception as e:
                      rospy.logwarn("NEPI_DRV: Failed to get Discovery info from module: " + f + " with exception: " + str(e))
                else:
                    rospy.logwarn("NEPI_DRV: Failed to get valid FILE_TYPE from: " + f )
                if open_success:
                  try:
                    #sys.modules.pop(module)
                    if module_name in sys.modules:
                      del sys.modules[module_name]
                    del module
                  except:
                    rospy.loginfo("NEPI_DRV: Failed to remove module: " + f)
    else:
        rospy.logwarn("NEPI_DRV: Driver path %s does not exist",  search_path)
    drvs_dict = dict()
    pkg_names = list(node_dict.keys())
    driver_pkg_names = list(driver_dict.keys())
    discovery_pkg_names = list(discovery_dict.keys())
    '''
    rospy.logwarn("NEPI_DRV: pkg_names: " + str(pkg_names))
    rospy.logwarn("NEPI_DRV: driver_pkg_names: " + str(driver_pkg_names))
    rospy.logwarn("NEPI_DRV: discovery_pkg_names: " + str(discovery_pkg_names))
    '''
    users_dict = dict()
    for pkg_name in pkg_names:
      users_dict[pkg_name] = []
    for pkg_name in pkg_names:
      drv_dict= dict()
      driver_pkg_name = node_dict[pkg_name]['driver_pkg_name']
      discovery_pkg_name = node_dict[pkg_name]['discovery_pkg_name']
      valid_driver = (driver_pkg_name == 'None' or driver_pkg_name in driver_pkg_names)
      valid_discovery = (discovery_pkg_name == 'None' or discovery_pkg_name in discovery_pkg_names)   
      '''  
      rospy.logwarn("***********************")
      rospy.logwarn("NEPI_DRV:node name " + pkg_name)
      rospy.logwarn("NEPI_DRV: driver name: " + driver_pkg_name)
      rospy.logwarn("NEPI_DRV: valid driver: " + str(valid_driver))
      rospy.logwarn("NEPI_DRV: discovery name " + discovery_pkg_name)
      rospy.logwarn("NEPI_DRV: valid discovery " + str(valid_discovery))
      '''
      if valid_driver:
        if valid_discovery: 
          valid_node_dict = False
          try:
            drv_dict['description'] = node_dict[pkg_name]['description']
            drv_dict['group'] = node_dict[pkg_name]['group']
            drv_dict['group_id'] = node_dict[pkg_name]['group_id']
            valid_node_dict = True

            if valid_node_dict:
              drv_dict['NODE_DICT'] = node_dict[pkg_name]
              # Update driver and discovery info
              if (driver_pkg_name != 'None'):
                drv_dict['DRIVER_DICT'] = driver_dict[driver_pkg_name]
              else:
                drv_dict['DRIVER_DICT'] = dict(init = "Init")
              # Update discovery info
              if (discovery_pkg_name != 'None'):
                drv_dict['DISCOVERY_DICT'] = discovery_dict[discovery_pkg_name]
              else:
                drv_dict['DISCOVERY_DICT'] = dict(init = "Init")

              drv_dict['DEVICE_DICT'] = dict(init = "Init")

              # Add some defualt dict values
              drv_dict['path'] = search_path
              drv_dict['order'] = -1
              drv_dict['active'] = False
              drv_dict['msg'] = ""
              success = True
          except Exception as e:
            rospy.logwarn("NEPI_DRV: Driver Node %s has invalid info in headers %s",  pkg_name, str(e))
            success = False
          if success:
            drvs_dict[pkg_name] = drv_dict

          # Add dependancy on other drivers to users dict
          if driver_pkg_name != 'None' and driver_pkg_name != pkg_name:
            users_dict[driver_pkg_name].append(pkg_name)
          if discovery_pkg_name != "None" and discovery_pkg_name != driver_pkg_name and discovery_pkg_name != pkg_name:
            users_dict[discovery_pkg_name].append(pkg_name)
        else:
          rospy.logwarn("NEPI_DRV: Driver Node %s has invalid discovery package",  pkg_name)
      else:
        rospy.logwarn("NEPI_DRV: Driver Node %s has invalid driver package",  pkg_name)
    # Now update from users dict
    for drv_name in drvs_dict.keys():
      for key in users_dict.keys():
        if drv_name == key:
          drvs_dict[drv_name]['users'] = users_dict[key]
    # Now assign factory orders
    drvs_dict = setFactoryDriverOrder(drvs_dict)
    return drvs_dict


# ln = sys._getframe().f_lineno ; self.printND(drvs_dict,ln)
def printDict(drvs_dict):
  rospy.logwarn('NEPI_DRV: ')
  rospy.logwarn('NEPI_DRV:*******************')
  if line_num is not None:
    rospy.logwarn('NEPI_DRV: ' + str(line_num))
  rospy.logwarn('NEPI_DRV: Printing Drv Driver Dictionary')

  for drv_name in drvs_dict.keys():
    drv_dict = drvs_dict[drv_name]
    rospy.logwarn('NEPI_DRV: ')
    rospy.logwarn('NEPI_DRV: ' + drv_name)
    rospy.logwarn(str(drv_dict))


def updateDriversDict(drivers_path,drvs_dict):
  success = True
  if drivers_path[-1] == "/":
      drivers_path = drivers_path[:-1]
  get_drvs_dict = getDriversDict(drivers_path)
  purge_list = []
  for drv_name in drvs_dict.keys():
    if drv_name not in get_drvs_dict.keys():
      purge_list.append(drv_name)
  for drv_name in purge_list:
    del drvs_dict[drv_name]

  for drv_name in get_drvs_dict.keys():
    if drv_name not in drvs_dict.keys():
      drvs_dict[drv_name] = get_drvs_dict[drv_name]
      drvs_dict[drv_name]['active'] = True
      drvs_dict = moveDriverBottom(drv_name,drvs_dict)
  return drvs_dict


def refreshDriversDict(drivers_path,drvs_dict):
  success = True
  if drivers_path[-1] == "/":
      drivers_path = drivers_path[:-1]
  get_drvs_dict = getDriversDict(drivers_path)
  for drv_name in get_drvs_dict.keys():
    if drv_name not in drvs_dict.keys():
      if drvs_dict[drv_name]['NODE_DICT']["discovery_pkg_name"] != "None":
        current_set_val = drvs_dict[drv_name]['DISCOVERY_DICT']['option_1_dict']['set_val']
        get_drvs_dict[drv_name]['DISCOVERY_DICT']['option_1_dict']['set_val'] = current_set_val
        current_set_val = drvs_dict[drv_name]['DISCOVERY_DICT']['option_2_dict']['set_val']
        get_drvs_dict[drv_name]['DISCOVERY_DICT']['option_2_dict']['set_val'] = current_set_val
      get_drvs_dict[drv_name]['order'] = drvs_dict[drv_name]['order']
      get_drvs_dict[drv_name]['active'] = drvs_dict[drv_name]['active']
  return drvs_dict


def getDriversByActive(drvs_dict):
  active_dict = dict()
  for drv_name in drvs_dict.keys():
    drv_dict = drvs_dict[drv_name]
    driver_active = drv_dict['active']
    if driver_active == True:
      active_dict[drv_name] = drv_dict
  return active_dict


def getDriversByGroup(group,drvs_dict):
  group_dict = dict()
  for drv_name in drvs_dict.keys():
    drv_dict = drvs_dict[drv_name]
    driver_group = drv_dict['group']
    if driver_group == group:
      group_dict[drv_name] = drv_dict
  return group_dict

def getDriversByGroupId(group_id,drvs_dict):
  group_id_dict = dict()
  for drv_name in drvs_dict.keys():
    drv_dict = drvs_dict[drv_name]
    driver_group_id = drv_dict['group_id']
    if driver_group_id == group_id:
      group_id_dict[drv_name] = drv_dict
  return group_id_dict


def setFactoryDriverOrder(drvs_dict):
  indexes = []
  factory_indexes = []
  man_ind = 0
  call_ind = 1000
  run_ind = call_ind * 1000
  launch_ind = run_ind * 1000
  catch_ind = launch_ind * 1000
  order = catch_ind
  for drv_name in drvs_dict.keys():
    drv_dict = drvs_dict[drv_name]
    if drvs_dict[drv_name]['NODE_DICT']["discovery_pkg_name"] != "None":
      discovery_pkg = drv_dict['DISCOVERY_DICT']['module_name']
      discovery_meth = drv_dict['DISCOVERY_DICT']['method']
      discovery_proc = drv_dict['DISCOVERY_DICT']['process']
      if discovery_meth == 'MANUAL':
        order = man_ind
      elif discovery_pkg != 'None' and discovery_proc == 'CALL':
        order = call_ind
      elif discovery_pkg != 'None' and discovery_proc == 'RUN':
        order = run_ind 
      elif discovery_pkg != 'None' and discovery_proc == 'LAUNCH':
        order = launch_ind
    while order in indexes:
      order += 1
    indexes.append(order)
  for val in indexes:
      factory_indexes.append(0)
  sorted_indexes = list(np.sort(indexes))
  for i in range(len(indexes)):
     factory_indexes[i] = sorted_indexes.index(indexes[i])
  for i, drv_name in enumerate(drvs_dict.keys()):
    drvs_dict[drv_name]['order'] = factory_indexes[i]
  return drvs_dict


def moveDriverTop(drv_name,drvs_dict):
  if drv_name in drvs_dict.keys():
    current_ordered_list = getDriversOrderedList(drvs_dict)
    current_order = current_ordered_list.index(drv_name)
    if current_order > 0:
      new_order = 0
      drvs_dict = setDriverOrder(drv_name,new_order,drvs_dict)
  return drvs_dict

def moveDriverBottom(drv_name,drvs_dict):
  if drv_name in drvs_dict.keys():
    current_ordered_list = getDriversOrderedList(drvs_dict)
    current_order = current_ordered_list.index(drv_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = len(current_ordered_list) - 1
      drvs_dict = setDriverOrder(drv_name,new_order,drvs_dict)
  return drvs_dict

def moveDriverUp(drv_name,drvs_dict):
  if drv_name in drvs_dict.keys():
    current_ordered_list = getDriversOrderedList(drvs_dict)
    current_order = current_ordered_list.index(drv_name)
    if current_order > 0:
      new_order = current_order -1
      drvs_dict = setDriverOrder(drv_name,new_order,drvs_dict)
  return drvs_dict

def moveDriverDown(drv_name,drvs_dict):
  if drv_name in drvs_dict.keys():
    current_ordered_list = getDriversOrderedList(drvs_dict)
    current_order = current_ordered_list.index(drv_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = current_order + 1
      drvs_dict = setDriverOrder(drv_name,new_order,drvs_dict)
  return drvs_dict

def setDriverOrder(drv_name,new_order,drvs_dict):
  if drv_name in drvs_dict.keys():
    ordered_list = getDriversOrderedList(drvs_dict)
    current_order = ordered_list.index(drv_name)
    driver_entry = ordered_list.pop(current_order)
    ordered_list.insert(new_order,driver_entry)
    for drv_name in drvs_dict.keys():
      drvs_dict[drv_name]['order'] = ordered_list.index(drv_name)
  return drvs_dict

def setDriverMsg(drv_name,msg,drvs_dict):
  if drv_name in drvs_dict.keys():
    drvs_dict[drv_name]['msg'] = str(msg)
  return drvs_dict

    

def getDriversOrderedList(drvs_dict):
  name_list = []
  order_list = []
  ordered_name_list = []
  for drv_name in drvs_dict.keys():
    name_list.append(drv_name)
    drv_dict = drvs_dict[drv_name]
    order = drv_dict['order']
    if order == -1:
      order = 100000
    while(order in order_list):
      order += 0.1
    order_list.append(order)
  s = list(sorted(order_list))
  indexes = [s.index(x) for x in order_list]
  for val in order_list:
    ordered_name_list.append(0)
  for i,index in enumerate(indexes):
    ordered_name_list[index] = name_list[i]
  return ordered_name_list

def getDriversActiveOrderedList(drvs_dict):
  ordered_name_list = getDriversOrderedList(drvs_dict)
  #rospy.logwarn("NEPI_DRV: Drivers Ordered List: " + str(ordered_name_list))
  ordered_active_list =[]
  for drv_name in ordered_name_list:
    #rospy.logwarn("NEPI_DRV: Drivers Drv Entry: " + str(drvs_dict[drv_name]))
    active = drvs_dict[drv_name]['active']
    if active:
      ordered_active_list.append(drv_name)
  return ordered_active_list






def getDriverFilesList(drivers_path):
  drivers_list = []
  if drivers_path != '':
    if os.path.exists(drivers_path):
      [file_list, num_files] = nepi_ros.get_file_list(drivers_path,"py")
  for f in file_list:
    drivers_list.append(f.split(".")[0])
  return drivers_list
  
def getDriverPackagesList(install_path):
  pkg_list = []
  if install_path != '':
    if os.path.exists(install_path):
      [file_list, num_files] = nepi_ros.get_file_list(install_path,"zip")
  for pkg in file_list:
    pkg_list.append(os.path.basename(pkg))
  return pkg_list


 
def activateAllDrivers(drvs_dict):
  success = True
  for drv_name in drvs_dict.keys():
    drvs_dict = activateDriver(drv_name,drvs_dict)
  return drvs_dict

def activateDriver(drv_name,drvs_dict):
    if drv_name not in drvs_dict.keys():
      rospy.logwarn("NEPI_DRV: Driver %s for removal request does not exist", drv_name)
      return drvs_dict
    drvs_dict[drv_name]['active'] = True
    return drvs_dict

def disableAllDrivers(drvs_dict):
  success = True
  for drv_name in drvs_dict.keys():
    drvs_dict = disableDriver(drv_name,drvs_dict)
  return drvs_dict

def disableDriver(drv_name,drvs_dict):
    if drv_name not in drvs_dict.keys():
      rospy.logwarn("NEPI_DRV: Driver %s for removal request does not exist", drv_name)
      return drvs_dict
    drvs_dict[drv_name]['active'] = False
    return drvs_dict

def installDriverPkg(pkg_name,drvs_dict,install_from_path,install_to_path):
    success = True
    if install_from_path[-1] == "/":
      install_from_path = install_from_path[:-1]
    if install_to_path[-1] == "/":
      search_path = install_to_path[:-1]

    if os.path.exists(install_from_path) == False:
      rospy.logwarn("NEPI_DRV: Install package source folder does not exist %s", install_from_path)
      return False, drvs_dict
    if os.path.exists(install_to_path) == False:
      rospy.logwarn("NEPI_DRV: Install package destination folder does not exist %s", install_to_path)
      return False, drvs_dict
    pkg_list = getDriverPackagesList(install_from_path)
    if pkg_name not in pkg_list:
      rospy.logwarn("NEPI_DRV: Install package for %s not found in install folder %s", pkg_name, install_from_path)
      return False, drvs_dict
    os_user = getpass.getuser()
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_from_path)
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_to_path)
    pkg_path = install_from_path + "/" + pkg_name
    driver_path = install_to_path
    try:
      pkg = zipfile.ZipFile(pkg_path)
      pkg_files = pkg.namelist()

    except Exception as e:
      rospy.logwarn("NEPI_DRV: " + str(e))
      success = False
    if success:
      # Create a list of files
      driver_files = []
      for pkg_file in pkg_files:
        driver_file = driver_path + "/" + pkg_file
        driver_files.append(driver_file)
      for file in driver_files:
        if os.path.exists(file):
          try:
            os.remove(file)
          except Exception as e:
            success = False
            rospy.logwarn(str(e))
      if success:
        # Unzip the package to the Driver path
        with zipfile.ZipFile(pkg_path,"r") as zip_ref:
          zip_ref.extractall(driver_path)
        # Check for success
        for f in driver_files:
          if os.path.exists(f) == False:
            os.system('chown -R ' + 'nepi:nepi' + ' ' + f)
            success = False
    drvs_dict = updateDriversDict(driver_path,drvs_dict)
    return success, drvs_dict 



def removeDriver(drv_name,drvs_dict,backup_path = None):
    success = True   
    if drv_name not in drvs_dict.keys():
      rospy.logwarn("NEPI_DRV: Driver %s for removal request does not exist", drv_name)
      return False, drvs_dict
    drv_dict = drvs_dict[drv_name]

    node_pkg_name = drv_name
    driver_pkg_name = drv_dict['DRIVER_DICT']['pkg_name']
    discovery_pkg_name = drv_dict['DISCOVERY_DICT']['pkg_name']
    driver_pkg_names = [node_pkg_name,driver_pkg_name,discovery_pkg_name]

    driver_files = []
    driver_files.append(drv_dict['NODE_DICT']['file_name'])
    if driver_pkg_name != "None":
      driver_files.append(drv_dict['DRIVER_DICT']['file_name'])
    if discovery_pkg_name != "None":
      driver_files.append(drv_dict['DISCOVERY_DICT']['file_name'])

    path = drv_dict['path']

    os_user = getpass.getuser()
    driver_file_list = []
    for i,driver_file in enumerate(driver_files):
      if driver_file != 'None' and driver_pkg_names[i] == drv_name:
        file = driver_files[i]
        filepath = path + '/' + file
        if os.path.exists(filepath) == False:
          success = False
        if success:
          os.system('chown -R ' + 'nepi:nepi' + ' ' + path)
          driver_file_list.append(filepath)
          # Create an install package from driver files
    rospy.loginfo("NEPI_DRV: Removing driver files: " + str(driver_file_list))      
    if backup_path != None:
      if backup_path[-1] == "/":
        backup_path = backup_path[:-1]
      if os.path.exists(backup_path) == False:
        backup_path = None
      else:
        os.system('chown -R ' + 'nepi:nepi' + ' ' + backup_path)
        zip_file = backup_path + "/" + drv_name + ".zip"
        rospy.loginfo("NEPI_DRV: Backing up removed file to: " + zip_file)
        try:
          zip = zipfile.ZipFile(zip_file, "w", zipfile.ZIP_DEFLATED)
          for file_path in driver_file_list:
            zip.write(file_path, os.path.basename(file_path), compress_type=zipfile.ZIP_DEFLATED)
          zip.close()
          zip = None
        except Exception as e:
          rospy.logwarn("NEPI_DRV: Failed to backup driver: " + str(e))
          if os.path.exists(zip_file) == True:
            try:
              zip.close()
            except Exception as e:
              rospy.logwarn(str(e))
            try:
              os.remove(file_path)
            except Exception as e:
              rospy.logwarn(str(e))
        for file_path in driver_file_list:
          if os.path.exists(file_path) == True:
            try:
              os.remove(file_path)
            except Exception as e:
              success = False
              rospy.logwarn("NEPI_DRV: Failed to remove driver file: " + file_path + " " + str(e))

    if success:
      del drvs_dict[drv_name]
    return success, drvs_dict



def launchDriverNode(file_name, ros_node_name, device_path = None):
  sub_process = None
  msg = 'Success'
  success = False
  if device_path is None:
    device_node_run_cmd = ['rosrun', 'nepi_drivers', file_name, '__name:=' + ros_node_name]
  else:
    device_node_run_cmd = ['rosrun', 'nepi_drivers', file_name, '__name:=' + ros_node_name, '_device_path:=' + device_path]
  try:
    sub_process = subprocess.Popen(device_node_run_cmd)
    success = True
  except Exception as e:
    msg = str("Failed to launch node %s with exception: %s", ros_node_name, str(e))
    rospy.logwarn("NEPI_DRV: " + msg)
  if success: 
    if sub_process.poll() is not None:
      msg = ("Failed to start " + device_node_name + " via " + " ".join(x for x in device_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
      rospy.logerr(msg)
      sub_process = None
      success = False
  return success, msg, sub_process
  

def killDriverNode(node_namespace,sub_process):
    success = True
    node_name = node_namespace.split("/")[-1]
    node_namespace_list = nepi_ros.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])
    rospy.logwarn("NEPI_DRV: " + str(node_list))
    rospy.logwarn("NEPI_DRV: " + node_name)
    if node_name in node_list:
      rospy.logwarn("NEPI_DRV: Killing node: " + node_name)
      [kill_list,fail_list] = rosnode.kill_nodes([node_name])
      time.sleep(2)    
      # Next check running processes
      if sub_process.poll() is not None: 
        sub_process.terminate()
        terminate_timeout = 3
        while (terminate_timeout > 0):
          time.sleep(1)
          if sub_process.poll() is not None:
            terminate_timeout -= 1
            success = False
          else:
            success = True
            break
        if success == False:
          # Escalate it
          sub_process.kill()
          time.sleep(1)
        if sub_process.poll() is not None:
          success = False
        else:
          success = True
    if success:
      rospy.logwarn("NEPI_DRV: Killed node: " + node_name)
    else:
       rospy.logwarn("NEPI_DRV: Failed to kill node: " + node_name)
    return success
        

def importDriverClass(file_name,file_path,module_name,class_name):
      module_class = None
      success = False
      msg = "failed"
      file_list = os.listdir(file_path)
      if file_name in file_list:
        sys.path.append(file_path)
        try:
          module = importlib.import_module(module_name)
          try:
            module_class = getattr(module, class_name)
            success = True
            msg = 'success'
          except Exception as e:
            rospy.logwarn("NEPI_DRV: Failed to import class %s from module %s with exception: %s", class_name, module_name, str(e))
        except Exception as e:
            rospy.logwarn("NEPI_DRV: Failed to import module %s with exception: %s", module_name, str(e))
      else:
        rospy.logwarn("NEPI_DRV: Failed to find file %s in path %s for module %s", file_name, file_path, module_name)
      return success, msg, module_class


def unimportDriverClass(module_name):
    success = True
    if module_name in sys.modules:
        try:
           sys.modules.pop(module_name)
        except:
            rospy.loginfo("NEPI_DRV: Failed to clordered_unimport module: " + module_name)
        if module_name in sys.modules:
          success = False
    return success


#######################
### Serial Port Utility Functions

STANDARD_BUAD_RATES = [110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

SERIAL_PORT_DICT_ENTRY = dict(    vendor_id = 0,
                        product_id = 0,
                        manf_str = "None",
                        buad_rates = []
                   )
                  

def getSerialPortDict():
  port_dict = dict()

  devs = usb.core.find(find_all=True)
  port = list_ports.comports()
  product_id = 0
  for port in sorted(port):
    entry = copy.deepcopy(SERIAL_PORT_DICT_ENTRY)
    entry["vender_id"] = port.vid
    for dev in devs:
      if dev.idVendor == port.vid:
        product_id = dev.idProduct
        break
    entry["product_id"] = product_id
    entry["manf_str"] = port.manufacturer
    port_dict[port.device] = entry
  return port_dict

### Function for checking if port is available
def checkSerialPorts(port_str):
    success = False
    port = list_ports.comports()
    for loc, desc, hwid in sorted(port):
      if loc == port_str:
        success = True
    return success



#***************************
# IDX utility functions

#Factory Control Values 
DEFAULT_CONTROLS_DICT = dict( controls_enable = True,
    auto_adjust = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.5,
    resolution_mode = 1, # LOW, MED, HIGH, MAX
    framerate_mode = 1, # LOW, MED, HIGH, MAX
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    min_range_m = 0.0,
    max_range_m = 1.0,
    zoom_ratio = 0.5, 
    rotate_ratio = 0.5,
    frame_3d = 'nepi_center_frame'
    )

def applyIDXControls2Image(cv2_img,IDXcontrols_dict=DEFAULT_CONTROLS_DICT,current_fps=20):
    if IDXcontrols_dict.get("controls_enable"): 
        resolution_ratio = IDXcontrols_dict.get("resolution_mode")/3
        [cv2_img,new_res] = nepi_img.adjust_resolution(cv2_img, resolution_ratio)
        if IDXcontrols_dict.get("auto_adjust") is False:
            cv2_img = nepi_img.adjust_brightness(cv2_img,IDXcontrols_dict.get("brightness_ratio"))
            cv2_img = nepi_img.adjust_contrast(cv2_img,IDXcontrols_dict.get("contrast_ratio"))
            cv2_img = nepi_img.adjust_sharpness(cv2_img,IDXcontrols_dict.get("threshold_ratio"))
        else:
            cv2_img = nepi_img.adjust_auto(cv2_img,0.3)
        ##  Need to get current framerate setting
        ##  Hard Coded for now
        framerate_ratio = IDXcontrols_dict.get("framerate_mode")/3
        [cv2_img,new_rate] = nepi_img.adjust_framerate(cv2_img, current_fps, framerate_ratio)
    return cv2_img

  
