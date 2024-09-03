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
import importlib
import rospy
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

DRIVER_DIR = '/opt/nepi/ros/lib/drivers'
DRIVER_FILE_TYPES = ['Node','Driver', 'Discovery']
DRIVER_KEYS = ['node','driver','discovery']


def getDriverDict(search_path):
    node_dict = dict()
    discovery_dict = dict()
    driver_dict = dict()
    file_list = []
    # Find driver files
    ind = 0
    if os.path.exits_list(search_path):
        if search_path[-1] != "/":
            search_path = search_path + "/"
        sys.path.append(search_path)
        rospy.loginfo("NEPI_NEX: Searching for drivers in path: " + search_path)
        for f in os.listdir(search_path):
          if f.endswith(".py"): 
            module_name = f.split(".")[0]
            rospy.logwarn("NEPI_NEX: Will try to import driver: " + module_name)
            open_success = True
            read_success = True
            try:
              module = __import__(module_name)
            except Exception as e:
              rospy.logwarn("NEPI_NEX: failed to import module %s with exception %s", f, str(e))
              open_succes = False
            if open_success:
                try:
                  module_type = module.FILE_TYPE                
                except:
                  rospy.logwarn("NEPI_NEX: No FILE_TYPE in module: " + f)
                  read_success = False
                if read_success:
                  if module_type == "NODE":
                    try:
                      node_name = module.DRIVER_NAME
                      node_dict[node_name] = {
                        'group': module.GROUP,
                        'group_id': module.GROUP_ID,
                        'node_file_name': f,
                        'node_file_path': search_path,
                        'node_module_name': module_name,
                        'node_class_name': module.CLASS_NAME,
                        'driver_name': module.DRIVER_NAME,
                        'driver_interfaces': module.DRIVER_INTERFACES,
                        'driver_options': module.DRIVER_OPTIONS,
                        'driver_default_option': module.DRIVER_DEFAULT_OPTION,
                        'driver_set_option': module.DRIVER_DEFAULT_OPTION,
                        'discovery_name': module.DISCOVERY_NAME, 
                        'discovery_method': module.DISCOVERY_METHOD, 
                        'order': -1
                        }
                      file_list.append(f)
                    except Exception as e:
                      rospy.logwarn("NEPI_NEX: Failed to get Node info from module: " + f +" with exception: " + str(e))

                  elif module_type == "DRIVER":
                    try:
                      driver_name = module.DRIVER_NAME
                      driver_dict[driver_name] = {
                        'file_name': f,
                        'file_path': search_path,
                        'module_name': module_name,
                        'class_name': module.CLASS_NAME,
                      }
                      file_list.append(f)
                    except Exception as e:
                      rospy.logwarn("NEPI_NEX: Failed to get Discovery info from module: " + f +" with exception: " + str(e))
                  elif module_type == "DISCOVERY":
                    try:
                      discovery_name = module.DRIVER_NAME
                      discovery_dict[module_name] = {
                        'file_name': f,
                        'file_path': search_path,
                        'module_name': module_name,
                        'class_name': module.CLASS_NAME,
                        'process': module.PROCESS
                      }
                      file_list.append(f)
                    except Exception as e:
                      rospy.logwarn("NEPI_NEX: Failed to get Discovery info from module: " + f + " with exception: " + str(e))
                else:
                    rospy.logwarn("NEPI_NEX: Failed to get valid FILE_TYPE from: " + f )
                if open_success:
                  try:
                    sys.modules.pop(module_name)
                  except:
                    rospy.loginfo("NEPI_NEX: Failed to clordered_liste  module: " + f)
    else:
        rospy.logwarn("NEPI_NEX: Driver path %s does not exist",  search_path)
    drivers_dict = dict()
    node_names = list(node_dict.keys())
    discovery_names = list(discovery_dict.keys())
    driver_names = list(driver_dict.keys())
    users_dict = dict()
    for node_name in nodes:
      users_dict[node_name] = []
    purge_list = []
    for node_name in node_names:
      driver_name = node['driver_name']
      discovery_name = node['discovery_name']
      valid_driver = (driver_name == 'None' or driver_name in driver_names)
      valid_discovery = (discovery_name == 'None' or discovery_name in discovery_names)
      if valid_driver and valid_discovery: 
        drivers_dict[node_name] = node_dict[node_name]
        # Update driver info
        if (driver_name != 'None'):
          drivers_dict[node_name]['driver_file_name'] = driver_dict[driver_name]['file_name']
          drivers_dict[node_name]['driver_file_path'] = driver_dict[driver_name]['file_path']
          drivers_dict[node_name]['driver_module_name'] = driver_dict[driver_name]['module_name']
          drivers_dict[node_name]['driver_class_name'] = driver_dict[driver_name]['class_name']
        else:
          drivers_dict[node_name]['driver_file_name'] = "None"
          drivers_dict[node_name]['driver_file_path'] = "None"
          drivers_dict[node_name]['driver_module_name'] = "None"
          drivers_dict[node_name]['driver_class_name'] = "None"
        # Update discovery info
        if (discovery_name != 'None'):
          drivers_dict[node_name]['discovery_file_name'] = discovery_dict[discovery_name]['file_name']
          drivers_dict[node_name]['discovery_file_path'] = discovery_dict[discovery_name]['file_path']
          drivers_dict[node_name]['discovery_module_name'] = discovery_dict[discovery_name]['module_name']
          drivers_dict[node_name]['discovery_class_name'] = discovery_dict[discovery_name]['class_name']
          drivers_dict[node_name]['discovery_process'] = discovery_dict[discovery_name]['process']
        else:
          drivers_dict[node_name]['discovery_file_name'] = "None"
          drivers_dict[node_name]['discovery_file_path'] = "None"
          drivers_dict[node_name]['discovery_module_name'] = "None"
          drivers_dict[node_name]['discovery_class_name'] = "None"
          drivers_dict[node_name]['discovery_process'] = "None"
        # Check if active
        active_path = search_path + 'active_drivers/'
        active_node_file =  active_path + node_dict[node_name]['node_file_name'] 
        node_active = os.path.exists(active_node_file)
        driver_active = True
        if node_dict[node_name]['driver_file_name'] != 'None':
          active_driver_file = active_path = active_path + node_dict[node_name]['driver_file_name'] 
          driver_active = os.path.exists(active_driver_file)
        if node_dict[node_name]['discovery_file_name'] != 'None':
          active_disc_file = active_path = active_path + node_dict[node_name]['discovery_file_name'] 
          disc_active = os.path.exists(active_disc_file)
        active = node_active and driver_active and disc_active
        drivers_dict[node_name]['active'] = active
        # Add dependancy on other drivers to users dict
        if driver_name != node_name:
          users_dict[driver_name].append(node_name)
        if discovery_name != node_name and discovery_name != driver_name:
          users_dict[discovery_name].append(node_name)
      else:
        rospy.logwarn("NEPI_NEX: Driver %s has invalid driver or discovery reference. Purging from drivers_dict",  node_name)
        purge_list.append(node_name)
    # Now update from users dict
    for node_name in node_names:
      drivers_dict[node_name]['users'] = users_dict[node_name]
    # Now purge bad drivers
    for node_name in purge_list:
      del drivers_dict[node_name]
    # Now assign factory orders
    drivers_dict = setFactoryDriverOrder(drivers_dict)
    return drivers_dict

def updateDriversDict(drivers_path,drivers_dict):
  success = True
  get_drivers_dict = getDriverDict(drivers_path)
  active_drivers_list = getActiveDriversList(drivers_path)
  purge_list = []
  for driver_name in drivers_dict.keys():
    if driver_name not in get_drivers_dict.keys():
      purge_list.append(driver_name)
    else:
      driver_dict = drivers_dict[driver_name]
      get_driver_dict = get_drivers_dict[driver_name]

      # Update active drivers
      desired_active_state = driver_dict['active']
      current_active_state = get_driver_dict['active']
      if desired_active_state != current_active_state:
        if desired_active_state == True:
          [success,drivers_dict] = activateDriver(driver_name,drivers_dict)
        else:
          [success,drivers_dict] = disableDriver(driver_name,drivers_dict)
  return drivers_dict


def getDriversByGroup(group,drivers_dict):
  group_dict = dict()
  for driver_name in drivers_dict.keys():
    driver = drivers_dict[driver_name]
    driver_group = driver['group']
    if driver_group == group:
      group_dict.append(driver)
  return group_dict

def getDriversByGroupId(group_id,drivers_dict):
  group_id_dict = dict()
  for driver_name in drivers_dict.keys():
    driver = drivers_dict[driver_name]
    driver_group_id = driver['group_id']
    if driver_group_id == group_id:
      group_id_dict.append(driver)
  return group_id_dict


def setFactoryDriverOrder(drivers_dict):
  indexes = []
  factory_indexes = []
  man_ind = 0
  call_ind = 1000
  run_ind = call_ind * 1000
  launch_ind = run_ind * 1000
  catch_ind = launch_ind * 1000
  for driver_name in drivers_dict.keys():
    driver = drivers_dict[driver_name]
    disc_name = driver['discovery_module_name']
    disc_meth = driver['discovery_method']
    disc_proc = driver['discovery_process']
    if disc_meth == 'MANUAL':
      order = man_ind
    elif disc_name != 'None' and disc_proc == 'CALL':
      order = call_ind
    elif disc_name != 'None' and disc_proc == 'RUN':
      order = run_ind 
    elif disc_name != 'None' and disc_proc == 'LAUNCH':
      order = launch_ind
    else:
      order = catch_ind
    while order in factory_indexes:
      order += 1
    indexes.append(order)
  for val in indexes:
      factory_indexes.append(0)
  sorted_indexes = list(np.sort(indexes))
  for i in range(len(indexes)):
     factory_indexes[i] = sorted_indexes.index[indexes[i]]
  drivers_dict[driver_name]['order'] = order
  return drivers_dict


def moveDriverTop(driver_name,drivers_dict):
  if driver_name in drivers_dict.keys():
    current_ordered_list = getDriversOrderedList(drivers_dict)
    current_order = current_ordered_list.index(driver_name)
    if current_order > 0:
      new_order = 0
      drivers_dict = setDriverOrder(driver_name,new_order,drivers_dict)
  return drivers_dict

def moveDriverBottom(driver_name,drivers_dict):
  if driver_name in drivers_dict.keys():
    current_ordered_list = getDriversOrderedList(drivers_dict)
    current_order = current_ordered_list.index(driver_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = len(current_ordered_list) - 1
      drivers_dict = setDriverOrder(driver_name,new_order,drivers_dict)
  return drivers_dict

def moveDriverUp(driver_name,drivers_dict):
  if driver_name in drivers_dict.keys():
    current_ordered_list = getDriversOrderedList(drivers_dict)
    current_order = current_ordered_list.index(driver_name)
    if current_order > 0:
      new_order = current_order -1
      drivers_dict = setDriverOrder(driver_name,new_order,drivers_dict)
  return drivers_dict

def moveDriverDown(driver_name,drivers_dict):
  if driver_name in drivers_dict.keys():
    current_ordered_list = getDriversOrderedList(drivers_dict)
    current_order = current_ordered_list.index(driver_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = current_order + 1
      drivers_dict = setDriverOrder(driver_name,new_order,drivers_dict)
  return drivers_dict

def setDriverOrder(driver_name,new_order,drivers_dict):
  if driver_name in drivers_dict.keys():
    ordered_list = getDriversOrderedList(drivers_dict)
    current_order = ordered_list.index(driver_name)
    driver_entry = ordered_list.pop(current_order)
    ordered_list.insert(new_order,driver_entry)
    for driver_name in drivers_dict.keys():
      drivers_dict[driver_name]['order'] = ordered_list.index(driver_name)
  return drivers_dict

    

def getDriversOrderedList(drivers_dict):
  name_list = []
  order_list = []
  for driver_name in drivers_dict.keys():
    name_list.append(driver_name)
    driver_dict = drivers_dict[driver_name]
    order = driver_dict['order']
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

  def getDriversActiveOrderedList(drivers_dict):
    ordered_name_list = getDriversOrderedList(drivers_dict)
    ordered_active_list =[]
    for driver_name in ordered_name_list:
      active = drivers_dict[driver_name]['active']
      if active:
        ordered_active_list.append(driver_name)
    return ordered_active_list
  


def updateDriverOrder(driver_name,driver_order,drivers_dict):
  success = True
  for driver_name in drivers_dict.keys():
    driver_dict = drivers_dict[driver_name]

    node_path = driver_dict['node_file_path']
    dri_path = driver_dict['driver_file_path']
    dis_path = driver_dict['discovery_file_path']
    driver_files = [node_path,dri_path,dis_path]

    for i,src in enumerate(driver_files):
      if src != "None":
          folder = os.path.dirname(src) + '/'
          filename = os.path.basename(src)
          link = folder + 'active_drivers/' + filename
          if os.path.exits_list(link) == True:
            success = False
  drivers_dict = drivers_dict[driver_name]['active'] = success
  return drivers_dict


def activateAllDrivers(drivers_dict):
  success = True
  for driver_name in drivers_dict.keys():
    [ret,drivers_dict] = activateDriver(driver_name,drivers_dict)
    if ret == False:
      success = False
  return success, drivers_dict

  
def activateDriver(driver_name,drivers_dict):
    success = True
    if driver_name not in drivers_dict.keys():
      rospy.logwarn("NEPI_NEX: Driver %s for removal request does not exist", driver_name)
      return success, drivers_dict
    driver_dict = drivers_dict[driver_name]

    node_name = driver_dict['node_name']
    dri_name = driver_dict['driver_name']
    dis_name = driver_dict['discovery_name']
    driver_names = [node_name,dri_name,dis_name]

    node_path = driver_dict['node_file_path']
    dri_path = driver_dict['driver_file_path']
    dis_path = driver_dict['discovery_file_path']
    driver_files = [node_path,dri_path,dis_path]

    for i,src in enumerate(driver_files):
      if src != 'None':
        if src == link:
          success = False
        else:
          folder = os.path.dirname(src) + '/'
          filename = os.path.basename(src)
          link = folder + 'active_drivers/' + filename
          if os.path.exits_list(link):
            try:
              ordered_name_list.remove(link)
            except Exception as e:
              success = False
              rospy.logwarn(str(e))
          if success:
            try:
              ordered_name_list.symlink(src,link)
            except Exception as e:
              rospy.logwarn(str(e))
            if os.path.exits_list(link) == False:
              success = False
    drivers_dict = drivers_dict[driver_name]['active'] = success
    return drivers_dict

def disableDriver(driver_name,drivers_dict):
    success = False
    link_success = True
    if driver_name not in drivers_dict.keys():
      rospy.logwarn("NEPI_NEX: Driver %s for removal request does not exist", driver_name)
      return success
    driver_dict = drivers_dict[driver_name]

    node_name = driver_dict['node_name']
    dri_name = driver_dict['driver_name']
    dis_name = driver_dict['discovery_name']
    driver_names = [node_name,dri_name,dis_name]

    node_path = driver_dict['node_file_path']
    dri_path = driver_dict['driver_file_path']
    dis_path = driver_dict['discovery_file_path']
    driver_files = [node_path,dri_path,dis_path]

    for i,src in enumerate(driver_files):
      if src != 'None' and driver_names[i] == driver_name:
        if os.path.exits_list(src):
          # Try and remove active path link
          folder = os.path.dirname(src) + '/'
          filename = os.path.basename(src)
          link = folder + 'active_drivers/' + filename
          if os.path.exits_list(link):
              try:
                ordered_name_list.remove(link)
                link_success = False
              except Exception as e:
                rospy.logwarn(str(e))
          else:
              link_success = False
    success = link_success
    drivers_dict = drivers_dict[driver_name]['active'] = False
    return drivers_dict

def removeDriver(driver_name,drivers_dict):
    success = False
    src_success = True
    link_success = True
    if driver_name not in drivers_dict.keys():
      rospy.logwarn("NEPI_NEX: Driver %s for removal request does not exist", driver_name)
      return success
    driver_dict = drivers_dict[driver_name]

    node_name = driver_dict['node_name']
    dri_name = driver_dict['driver_name']
    dis_name = driver_dict['discovery_name']
    driver_names = [node_name,dri_name,dis_name]

    node_path = driver_dict['node_file_path']
    dri_path = driver_dict['driver_file_path']
    dis_path = driver_dict['discovery_file_path']
    driver_files = [node_path,dri_path,dis_path]

    for i,src in enumerate(driver_files):
      if src != 'None' and driver_names[i] == driver_name:
        if os.path.exits_list(src):
          # Try and remove active path link first
          folder = os.path.dirname(src) + '/'
          filename = os.path.basename(src)
          link = folder + 'active_drivers/' + filename
          if os.path.exits_list(link):
              try:
                ordered_name_list.remove(link)
                link_success = False
              except Exception as e:
                rospy.logwarn(str(e))
          else:
              link_success = False
          # Then remove the src file
          try:
            ordered_name_list.remove(source)
          except Exception as e:
            src_success = False
            rospy.logwarn(str(e))
    success = src_success and link_success
    if success:
      del drivers_dict[driver_name]
    return drivers_dict


def getActiveDriversList(drivers_path):
  active_list = nepi_ros.get_file_list(drivers_path)
  return active_list


def LaunchDriverNode(driver_name, drivers_dict, ros_node_name):
  package_folder = 'nepi_drivers'
  file_name = drivers_dict[driver_name]['node_file_name']
  subprocess = LaunchNode(package_folder, file_name, ros_node_name)
  return subprocess

def LaunchDriverDisovery(driver_name, drivers_dict, ros_node_name):
  subprocess = None
  package_folder = 'nepi_drivers'
  file_name = drivers_dict[driver_name]['discovery_file_name']
  disc_process = drivers_dict[driver_name]['discovery_process']
  if disc_process != "LAUNCH":
    rospy.logwarn("NEPI_NEX: Requested discovery launch file process is %s for driver %s, only LAUNCH process supported", disc_process, driver_name)
  subprocess = LaunchNode(package_folder, file_name, ros_node_name)
  return subprocess

def RunDriverDisovery(driver_name, drivers_dict):
  subprocess = None
  return subprocess # need to implement

def callDriverDisovery(driver_name, drivers_dict):
  return # need to implement


def importDriverNodeClass(driver_name,drivers_dict):
  node_class = importDriverClass(driver_name,drivers_dict,file_type='Node')
  return node_class

def importDriverDriverClass(driver_name,drivers_dict):
  driver_class = importDriverClass(driver_name,drivers_dict,file_type='Driver')
  return driver_class

def importDriverDiscovderyClass(driver_name,drivers_dict):
  discovery_class = importDriverClass(driver_name,drivers_dict,file_type='Discovery')
  return discovery_class

def importDriverClass(driver_name,drivers_dict,file_type='Node'):
    module_class = None
    if file_type not in DRIVER_FILE_TYPES:
      rospy.logwarn("NEPI_NEX: Invalid file_type requested on import: " + file_type)
      return module_class
    if driver_name not in drivers_dict.keys():
      rospy.logwarn("NEPI_NEX: Driver with driver_name: %s does not exsist", driver_name)
      return module_class
    driver_dict = drivers_dict[driver_name]
    file_key = DRIVER_KEYS[DRIVER_FILE_TYPES.index(file_type)]
    file_name = driver_dict[file_key + '_file_name']
    file_path = driver_dict[file_key + '_file_path']
    module_name = driver_dict[file_key + '_module_name']
    class_name = driver_dict[file_key + '_class_name']
    if file_name == 'None' or module_name == 'None' or class_name == 'None':
      rospy.logwarn("NEPI_NEX: Requested file_type %s for driver_name: %s does not exsist", file_type, driver_name)
      return module_class
    else:
      sys.path.append(file_path)
      try:
        module_class = importlib.import_module(module_name)
        try:
          my_class = getattr(module, class_name)
        except Exception as e:
          rospy.logwarn("NEPI_NEX: Failed to import class %s from module %s", class_name, module_name, str(e))
      except Exception as e:
          rospy.logwarn("NEPI_NEX: Failed to import module %s", module_name, str(e))
    return module_class


def unimportDriverClass(driver_name):
    success = True
    if driver_name in sys.modules:
        try:
           sys.modules.pop(driver_name)
        except:
            rospy.loginfo("NEPI_NEX: Failed to clordered_liste module: " + driver_name)
        if driver_name in sys.modules:
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
  port = list_ports.comport()
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
def checkSerialPortexits_list(port_str):
    success = False
    port = list_ports.comport()
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

  
