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
  
#***************************
# NEX Driver utility functions


CLASS = 'IDX'
SUBCLASS = 'GenICam'
TYPE = 'Node'
ID = 'GenicamCamDriver'
INTERFACES = ['USB','IP'] # 'USB','IP','SERIALUSB','SERIAL','CANBUS'
DISCOVERY = 'AUTO'  # 'AUTO' or 'MANUAL'
DISCOVERY_ID = 'IDXGenicamCamDiscovery' # 'Discovery File ID' or 'None'

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
        #rospy.loginfo("NEPI_NEX: Searching for drivers in path: " + search_path)
        for f in os.listdir(search_path):
          if f.endswith(".py"): 
            module_name = f.split(".")[0]
            #rospy.loginfo("NEPI_NEX: Will try to import module: " + module_name)
            open_success = True
            read_success = True
            warnings.filterwarnings('ignore', '.*unclosed.*', )
            try:
              module = __import__(module_name)
            except Exception as e:
              rospy.logwarn("NEPI_NEX: failed to import module %s with exception %s", f, str(e))
              open_success = False
            if open_success:
                try:
                  module_type = module.FILE_TYPE                
                except:
                  rospy.logwarn("NEPI_NEX: No FILE_TYPE in module: " + f)
                  read_success = False
                if read_success:
                  #rospy.logwarn("NEPI_NEX: " + module_type)
                  if module_type == "NODE":
                    try:
                      node_name = module.PKG_NAME
                      node_dict[node_name] = {
                        'description': module.DESCRIPTION,
                        'group': module.GROUP,
                        'group_id': module.GROUP_ID,
                        'node_file_name': f,
                        'node_file_path': search_path,
                        'node_module_name': module_name,
                        'node_class_name': module.CLASS_NAME,
                        'driver_name': module.DRIVER_PKG_NAME,
                        'driver_interfaces': module.DRIVER_INTERFACES,
                        'driver_options_1_name': module.DRIVER_OPTIONS_1_NAME,
                        'driver_options_1': module.DRIVER_OPTIONS_1,
                        'driver_default_option_1': module.DRIVER_DEFAULT_OPTION_1,
                        'driver_set_option_1': module.DRIVER_DEFAULT_OPTION_1,
                        'driver_options_2_name': module.DRIVER_OPTIONS_2_NAME,
                        'driver_options_2': module.DRIVER_OPTIONS_2,
                        'driver_default_option_2': module.DRIVER_DEFAULT_OPTION_2,
                        'driver_set_option_2': module.DRIVER_DEFAULT_OPTION_2,
                        'discovery_name': module.DISCOVERY_PKG_NAME, 
                        'discovery_method': module.DISCOVERY_METHOD, 
                        'discovery_ids': module.DISCOVERY_IDS,
                        'discovery_ignore_ids': module.DISCOVERY_IGNORE_IDS,
                        'order': -1,
                        'active': False,
                        'msg': ""
                        }
                    except Exception as e:
                      rospy.logwarn("NEPI_NEX: Failed to get Node info from module: " + f +" with exception: " + str(e))

                  elif module_type == "DRIVER":
                    try:
                      driver_name = module.PKG_NAME
                      driver_dict[driver_name] = {
                        'file_name': f,
                        'file_path': search_path,
                        'module_name': module_name,
                        'class_name': module.CLASS_NAME,
                      }
                    except Exception as e:
                      rospy.logwarn("NEPI_NEX: Failed to get Discovery info from module: " + f +" with exception: " + str(e))
                  elif module_type == "DISCOVERY":
                    try:
                      discovery_name = module.PKG_NAME
                      discovery_dict[discovery_name] = {
                        'file_name': f,
                        'file_path': search_path,
                        'module_name': module_name,
                        'class_name': module.CLASS_NAME,
                        'process': module.PROCESS
                      }
                    except Exception as e:
                      rospy.logwarn("NEPI_NEX: Failed to get Discovery info from module: " + f + " with exception: " + str(e))
                else:
                    rospy.logwarn("NEPI_NEX: Failed to get valid FILE_TYPE from: " + f )
                if open_success:
                  try:
                    #sys.modules.pop(module)
                    if module_name in sys.modules:
                      del sys.modules[module_name]
                    del module
                  except:
                    rospy.loginfo("NEPI_NEX: Failed to remove module: " + f)
    else:
        rospy.logwarn("NEPI_NEX: Driver path %s does not exist",  search_path)
    nex_database = dict()
    node_names = list(node_dict.keys())
    driver_names = list(driver_dict.keys())
    discovery_names = list(discovery_dict.keys())
    #rospy.logwarn("NEPI_NEX: node_names: " + str(node_names))
    #rospy.logwarn("NEPI_NEX: driver_names: " + str(driver_names))
    #rospy.logwarn("NEPI_NEX: discovery_names: " + str(discovery_names))
    users_dict = dict()
    for node_name in node_names:
      users_dict[node_name] = []
    for node_name in node_names:
      driver_name = node_dict[node_name]['driver_name']
      discovery_name = node_dict[node_name]['discovery_name']
      valid_driver = (driver_name == 'None' or driver_name in driver_names)
      valid_discovery = (discovery_name == 'None' or discovery_name in discovery_names)      
      #rospy.logwarn("***********************")
      #rospy.logwarn("NEPI_NEX:node name " + node_name)
      #rospy.logwarn("NEPI_NEX: driver name: " +driver_name)
      #rospy.logwarn("NEPI_NEX: valid driver: " + str(valid_driver))
      #rospy.logwarn("NEPI_NEX: discovery name " + discovery_name)
      #rospy.logwarn("NEPI_NEX: valid discovery " + str(valid_discovery))
      if valid_driver:
        if valid_discovery: 
          nex_database[node_name] = node_dict[node_name]
          # Update driver and discovery info
          if (driver_name != 'None'):
            nex_database[node_name]['driver_file_name'] = driver_dict[driver_name]['file_name']
            nex_database[node_name]['driver_file_path'] = driver_dict[driver_name]['file_path']
            nex_database[node_name]['driver_module_name'] = driver_dict[driver_name]['module_name']
            nex_database[node_name]['driver_class_name'] = driver_dict[driver_name]['class_name']
          else:
            nex_database[node_name]['driver_file_name'] = "None"
            nex_database[node_name]['driver_file_path'] = "None"
            nex_database[node_name]['driver_module_name'] = "None"
            nex_database[node_name]['driver_class_name'] = "None"
          # Update discovery info
          if (discovery_name != 'None'):
            nex_database[node_name]['discovery_file_name'] = discovery_dict[discovery_name]['file_name']
            nex_database[node_name]['discovery_file_path'] = discovery_dict[discovery_name]['file_path']
            nex_database[node_name]['discovery_module_name'] = discovery_dict[discovery_name]['module_name']
            nex_database[node_name]['discovery_class_name'] = discovery_dict[discovery_name]['class_name']
            nex_database[node_name]['discovery_process'] = discovery_dict[discovery_name]['process']
          else:
            nex_database[node_name]['discovery_file_name'] = "None"
            nex_database[node_name]['discovery_file_path'] = "None"
            nex_database[node_name]['discovery_module_name'] = "None"
            nex_database[node_name]['discovery_class_name'] = "None"
            nex_database[node_name]['discovery_process'] = "None"
          # Add dependancy on other drivers to users dict
          if driver_name != 'None' and driver_name != node_name:
            users_dict[driver_name].append(node_name)
          if discovery_name != "None" and discovery_name != driver_name and discovery_name != node_name:
            users_dict[discovery_name].append(node_name)
        else:
          rospy.logwarn("NEPI_NEX: Driver Node %s has invalid discovery info in header",  node_name)
      else:
        rospy.logwarn("NEPI_NEX: Driver Node %s has invalid driver info in header",  node_name)
    # Now update from users dict
    for nex_name in nex_database.keys():
      for key in users_dict.keys():
        if nex_name == key:
          nex_database[nex_name]['users'] = users_dict[key]
    # Now assign factory orders
    nex_database = setFactoryDriverOrder(nex_database)
    return nex_database


# ln = sys._getframe().f_lineno ; self.printND(nex_database,ln)
def printDict(nex_database):
  rospy.logwarn('NEPI_NEX: ')
  rospy.logwarn('NEPI_NEX:*******************')
  if line_num is not None:
    rospy.logwarn('NEPI_NEX: ' + str(line_num))
  rospy.logwarn('NEPI_NEX: Printing Nex Driver Dictionary')

  for nex_name in nex_database.keys():
    nex_dict = nex_database[nex_name]
    rospy.logwarn('NEPI_NEX: ')
    rospy.logwarn('NEPI_NEX: ' + nex_name)
    rospy.logwarn(str(nex_dict))


def updateDriversDict(drivers_path,nex_database):
  success = True
  if drivers_path[-1] == "/":
      search_path = drivers_path[:-1]
  get_nex_database = getDriversDict(drivers_path)
  purge_list = []
  for nex_name in nex_database.keys():
    if nex_name not in get_nex_database.keys():
      purge_list.append(nex_name)
  for nex_name in purge_list:
    del nex_database[nex_name]

  for nex_name in get_nex_database.keys():
    if nex_name not in nex_database.keys():
      nex_database[nex_name] = get_nex_database[nex_name]
      nex_database[nex_name]['active'] = True
      nex_database = moveDriverBottom(nex_name,nex_database)
  return nex_database


def refreshDriversDict(drivers_path,nex_database):
  success = True
  if drivers_path[-1] == "/":
      search_path = drivers_path[:-1]
  get_nex_database = getDriversDict(drivers_path)
  for nex_name in get_nex_database.keys():
    if nex_name not in nex_database.keys():
      get_nex_database[nex_name]['driver_set_option_1'] = nex_database[nex_name]['driver_set_option_1']
      get_nex_database[nex_name]['driver_set_option_2'] = nex_database[nex_name]['driver_set_option_2']
      get_nex_database[nex_name]['order'] = nex_database[nex_name]['order']
      get_nex_database[nex_name]['active'] = nex_database[nex_name]['active']
  return nex_database


def getDriversByActive(nex_database):
  active_dict = dict()
  for nex_name in nex_database.keys():
    nex_dict = nex_database[nex_name]
    driver_active = nex_dict['active']
    if driver_active == True:
      active_dict[nex_name] = nex_dict
  return active_dict


def getDriversByGroup(group,nex_database):
  group_dict = dict()
  for nex_name in nex_database.keys():
    nex_dict = nex_database[nex_name]
    driver_group = nex_dict['group']
    if driver_group == group:
      group_dict[nex_name] = nex_dict
  return group_dict

def getDriversByGroupId(group_id,nex_database):
  group_id_dict = dict()
  for nex_name in nex_database.keys():
    nex_dict = nex_database[nex_name]
    driver_group_id = nex_dict['group_id']
    if driver_group_id == group_id:
      group_id_dict[nex_name] = nex_dict
  return group_id_dict


def setFactoryDriverOrder(nex_database):
  indexes = []
  factory_indexes = []
  man_ind = 0
  call_ind = 1000
  run_ind = call_ind * 1000
  launch_ind = run_ind * 1000
  catch_ind = launch_ind * 1000
  for nex_name in nex_database.keys():
    nex_dict = nex_database[nex_name]
    discovery_name = nex_dict['discovery_module_name']
    disc_meth = nex_dict['discovery_method']
    disc_proc = nex_dict['discovery_process']
    if disc_meth == 'MANUAL':
      order = man_ind
    elif discovery_name != 'None' and disc_proc == 'CALL':
      order = call_ind
    elif discovery_name != 'None' and disc_proc == 'RUN':
      order = run_ind 
    elif discovery_name != 'None' and disc_proc == 'LAUNCH':
      order = launch_ind
    else:
      order = catch_ind
    while order in indexes:
      order += 1
    indexes.append(order)
  for val in indexes:
      factory_indexes.append(0)
  sorted_indexes = list(np.sort(indexes))
  for i in range(len(indexes)):
     factory_indexes[i] = sorted_indexes.index(indexes[i])
  for i, nex_name in enumerate(nex_database.keys()):
    nex_database[nex_name]['order'] = factory_indexes[i]
  return nex_database


def moveDriverTop(nex_name,nex_database):
  if nex_name in nex_database.keys():
    current_ordered_list = getDriversOrderedList(nex_database)
    current_order = current_ordered_list.index(nex_name)
    if current_order > 0:
      new_order = 0
      nex_database = setDriverOrder(nex_name,new_order,nex_database)
  return nex_database

def moveDriverBottom(nex_name,nex_database):
  if nex_name in nex_database.keys():
    current_ordered_list = getDriversOrderedList(nex_database)
    current_order = current_ordered_list.index(nex_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = len(current_ordered_list) - 1
      nex_database = setDriverOrder(nex_name,new_order,nex_database)
  return nex_database

def moveDriverUp(nex_name,nex_database):
  if nex_name in nex_database.keys():
    current_ordered_list = getDriversOrderedList(nex_database)
    current_order = current_ordered_list.index(nex_name)
    if current_order > 0:
      new_order = current_order -1
      nex_database = setDriverOrder(nex_name,new_order,nex_database)
  return nex_database

def moveDriverDown(nex_name,nex_database):
  if nex_name in nex_database.keys():
    current_ordered_list = getDriversOrderedList(nex_database)
    current_order = current_ordered_list.index(nex_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = current_order + 1
      nex_database = setDriverOrder(nex_name,new_order,nex_database)
  return nex_database

def setDriverOrder(nex_name,new_order,nex_database):
  if nex_name in nex_database.keys():
    ordered_list = getDriversOrderedList(nex_database)
    current_order = ordered_list.index(nex_name)
    driver_entry = ordered_list.pop(current_order)
    ordered_list.insert(new_order,driver_entry)
    for nex_name in nex_database.keys():
      nex_database[nex_name]['order'] = ordered_list.index(nex_name)
  return nex_database

def setDriverMsg(nex_name,msg,nex_database):
  if nex_name in nex_database.keys():
    nex_database[nex_name]['msg'] = str(msg)
  return nex_database

    

def getDriversOrderedList(nex_database):
  name_list = []
  order_list = []
  for nex_name in nex_database.keys():
    name_list.append(nex_name)
    nex_dict = nex_database[nex_name]
    order = nex_dict['order']
    if order == -1:
      order = 100000
    while(order in order_list):
      order += 0.1
    order_list.append(order)
    s = list(sorted(order_list))
    indexes = [s.index(x) for x in order_list]
    ordered_name_list = []
    for val in order_list:
      ordered_name_list.append(0)
    for i,index in enumerate(indexes):
      ordered_name_list[index] = name_list[i]
  return ordered_name_list

def getDriversActiveOrderedList(nex_database):
  ordered_name_list = getDriversOrderedList(nex_database)
  ordered_active_list =[]
  for nex_name in ordered_name_list:
    active = nex_database[nex_name]['active']
    if active:
      ordered_active_list.append(nex_name)
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


 
def activateAllDrivers(nex_database):
  success = True
  for nex_name in nex_database.keys():
    nex_database = activateDriver(nex_name,nex_database)
  return nex_database

def activateDriver(nex_name,nex_database):
    if nex_name not in nex_database.keys():
      rospy.logwarn("NEPI_NEX: Driver %s for removal request does not exist", nex_name)
      return nex_database
    nex_database[nex_name]['active'] = True
    return nex_database

def disableDriver(nex_name,nex_database):
    if nex_name not in nex_database.keys():
      rospy.logwarn("NEPI_NEX: Driver %s for removal request does not exist", nex_name)
      return nex_database
    nex_database[nex_name]['active'] = False
    return nex_database

def installDriverPkg(pkg_name,nex_database,install_from_path,install_to_path):
    success = True
    if os.path.exists(install_from_path) == False:
      rospy.logwarn("NEPI_NEX: Install package source folder does not exist %s", install_from_folder)
      return False, nex_database
    if os.path.exists(install_to_path) == False:
      rospy.logwarn("NEPI_NEX: Install package destination folder does not exist %s", install_to_folder)
      return False, nex_database
    pkg_list = getDriverPackagesList(install_from_path)
    if pkg_name not in pkg_list:
      rospy.logwarn("NEPI_NEX: Install package for %s not found in install folder %s", pkg_name, install_folder)
      return False, nex_database
    os_user = getpass.getuser()
    os.system('chown -R ' + os_user + ':' + os_user + ' ' + install_from_folder)
    os.system('chown -R ' + os_user + ':' + os_user + ' ' + install_to_folder)
    pkg_path = install_from_path + "/" + pkg_name + ".zip"
    driver_path = install_to_path
    pkg = zipfile.ZipFile(pkg_path)
    pkg_files = pkg.namelist()
    # Create a list of files
    driver_files = []
    for pkg_file in pkg_files:
      driver_file = driver_path + "/" + pkg_file
      driver_files.append(driver_file)
    for file in driver_files:
      if os.path.exsits(file):
        try:
          os.remove(file)
        except Exception as e:
          success = False
          rospy.logwarn(str(e))
    if success:
      # Unzip the package to the Driver path
      with zipfile.ZipFile(pkg,"r") as zip_ref:
        zip_ref.extractall(driver_path)
      # Check for success
      for f in drivers_files:
        if os.path.exists() == False:
          success = False
    nex_database = updateDriversDict(driver_path,nex_database)
    return success, nex_database 



def removeDriver(nex_name,nex_database,backup_path = None):
    success = True
    if nex_name not in nex_database.keys():
      rospy.logwarn("NEPI_NEX: Driver %s for removal request does not exist", nex_name)
      return False, nex_database
    nex_dict = nex_database[nex_name]

    node_name = nex_name
    driver_name = nex_dict['driver_name']
    discovery_name = nex_dict['discovery_name']
    driver_names = [node_name,driver_name,discovery_name]

    node_file = nex_dict['node_file_name']
    driv_file = nex_dict['driver_file_name']
    disc_file = nex_dict['discovery_file_name']

    node_path = nex_dict['node_file_path']
    driv_path = nex_dict['driver_file_path']
    disc_path = nex_dict['discovery_file_path']

    driver_files = [node_file, driv_file, disc_file]
    driver_paths = [node_path, driv_path, disc_path]

    os_user = getpass.getuser()
    driver_file_list = []
    for i,driver_file in enumerate(driver_files):
      if driver_file != 'None' and driver_names[i] == nex_name:
        path = driver_paths[i]
        file = driver_files[i]
        filepath = path + '/' + file
        driver_file_list.append(filepath)
        if os.path.exists(path) == False:
          success = False
        if success:
          os.system('chown -R ' + os_user + ':' + os_user + ' ' + path)
          # Create an install package from driver files
    if backup_path != None:
      if os.path.exists(backup_path):
        os.system('chown -R ' + os_user + ':' + os_user + ' ' + backup_path)
      else:
        backup_path = None

    if backup_path != None:
      zip_file = backup_path + "/" + nex_name + "zip"
      rospy.loginfo("NEPI_NEX: Backing up removed file to: " + zip_file)
      try:
        zip = zipfile.ZipFile(zip_file, "w", zipfile.ZIP_DEFLATED)
        for file in driver_file_list:
          zip.write(file)
        zip.close()
      except Exception as e:
        rospy.logwarn("NEPI_NEX: Failed to backup removed driver: " + str(e))
    

    for file in driver_file_list:
      try:
        os.remove(filepath)
      except Exception as e:
        success = False
        rospy.logwarn("NEPI_NEX: Failed to remove driver file: " + file + " " + str(e))

    if success:
      del nex_database[nex_name]
    return nex_database



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
    rospy.logwarn("NEPI_NEX: " + msg)
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
    rospy.logwarn("NEPI_NEX: " + str(node_list))
    rospy.logwarn("NEPI_NEX: " + node_name)
    if node_name in node_list:
      rospy.logwarn("NEPI_NEX: Killing node: " + node_name)
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
      rospy.logwarn("NEPI_NEX: Killed node: " + node_name)
    else:
       rospy.logwarn("NEPI_NEX: Failed to kill node: " + node_name)
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
            rospy.logwarn("NEPI_NEX: Failed to import class %s from module %s with exception: %s", class_name, module_name, str(e))
        except Exception as e:
            rospy.logwarn("NEPI_NEX: Failed to import module %s with exception: %s", module_name, str(e))
      else:
        rospy.logwarn("NEPI_NEX: Failed to find file %s in path %s for module %s", file_name, file_path, module_name)
      return success, msg, module_class


def unimportDriverClass(module_name):
    success = True
    if module_name in sys.modules:
        try:
           sys.modules.pop(module_name)
        except:
            rospy.loginfo("NEPI_NEX: Failed to clordered_unimport module: " + module_name)
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

  
