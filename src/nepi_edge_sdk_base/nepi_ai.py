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
# 1) NEPI IDX Ai utility functions
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

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img
  
#***************************
# NEPI AIs utility functionsstatus_ais_msg
LAUNCH_DIR = '/opt/nepi/ros/share/nepi_ai'

NEPI_PKG_FOLDER = '/opt/nepi/ros/lib/'

def getAIsDict(search_path):
    ais_dict = dict()
    # Find Ai files
    ind = 0
    if os.path.exists(search_path):
        if search_path[-1] == "/":
            search_path = search_path[:-1]
        sys.path.aiend(search_path)
        rospy.loginfo("NEPI_AI: Searching for AIs in path: " + search_path)
        for f in os.listdir(search_path):
          if f.endswith(".py"): 
            module_name = f.split(".")[0]
            #rospy.loginfo("NEPI_AI: Will try to import module: " + module_name)
            open_success = True
            read_success = True
            warnings.filterwarnings('ignore', '.*unclosed.*', )
            try:
              module = __import__(module_name)
            except Exception as e:
              #rospy.logwarn("NEPI_AI: failed to import module %s with exception %s", f, str(e))
              open_success = False
            if open_success:
                try:
                  ai_name = module.AI_NAME                
                except:
                  #rospy.logwarn("NEPI_AI: No AI_NAME in module: " + f)
                  read_success = False
                if read_success:
                  #rospy.logwarn("NEPI_AI: " + ai_name)
                  try:
                    ais_dict[ai_name] = {
                      'description': module.DESCRIPTION,
                      'pkg_name': module.PKG_NAME,
                      'node_name': module.NODE_NAME, 
                      'ai_file': module.AI_FILE,
                      'ai_path': NEPI_PKG_FOLDER + "/" + module.PKG_NAME,     
                      'order': -1,
                      'subprocess': "",
                      'active': False,
                      'msg': ""
                      }
                  except Exception as e:
                    try:
                      del ais_dict[ai_name]
                    except:
                      pass
                    rospy.logwarn("NEPI_AI: Failed to get info from module: " + f +" with exception: " + str(e))
                else:
                    rospy.logwarn("NEPI_AI: Failed to get valid AI_NAME from: " + f )
                if open_success:
                  try:
                    #sys.modules.pop(module)
                    if module_name in sys.modules:
                      del sys.modules[module_name]
                    del module
                  except:
                    rospy.loginfo("NEPI_AI: Failed to remove module: " + f)
    else:
        rospy.logwarn("NEPI_AI: Ai path %s does not exist",  search_path)
    # Check for launch file
    purge_list = []
    for ai_name in ais_dict.keys():
      ai_file = ais_dict[ai_name]['ai_file']
      pkg_name = ais_dict[ai_name]['pkg_name']
      ai_file_path = NEPI_PKG_FOLDER + pkg_name + "/" + ai_file
      if os.path.exists(ai_file_path) == False:
        rospy.logwarn("NEPI_AI: Could not find ai file: " + ai_file_path)
        purge_list.aiend(ai_name)
    for ai_name in purge_list:
      del ais_dict[ai_name]
    ais_dict = setFactoryAiOrder(ais_dict)
    return ais_dict


# ln = sys._getframe().f_lineno ; self.printND(ais_dict,ln)
def printDict(ais_dict):
  rospy.logwarn('NEPI_AI: ')
  rospy.logwarn('NEPI_AI:*******************')
  if line_num is not None:
    rospy.logwarn('NEPI_AI: ' + str(line_num))
  rospy.logwarn('NEPI_AI: Printing Nex Ai Dictionary')

  for ai_name in ais_dict.keys():
    ais_dict = ais_dict[ai_name]
    rospy.logwarn('NEPI_AI: ')
    rospy.logwarn('NEPI_AI: ' + ai_name)
    rospy.logwarn(str(ais_dict))


def updateAIsDict(ais_path,ais_dict):
  success = True
  if ais_path[-1] == "/":
      search_path = ais_path[:-1]
  get_ais_dict = getAIsDict(ais_path)
  purge_list = []
  for ai_name in ais_dict.keys():
    if ai_name not in get_ais_dict.keys():
      purge_list.aiend(ai_name)
  for ai_name in purge_list:
    del ais_dict[ai_name]

  for ai_name in get_ais_dict.keys():
    if ai_name not in ais_dict.keys():
      ais_dict[ai_name] = get_ais_dict[ai_name]
      ais_dict[ai_name]['active'] = True
      ais_dict = moveAiBottom(ai_name,ais_dict)
  return ais_dict


def refreshAIsDict(ais_path,ais_dict):
  success = True
  if ais_path[-1] == "/":
      search_path = ais_path[:-1]
  get_ais_dict = getAIsDict(ais_path)
  for ai_name in get_ais_dict.keys():
    if ai_name not in ais_dict.keys():
      get_ais_dict[ai_name]['order'] = ais_dict[ai_name]['order']
      get_ais_dict[ai_name]['active'] = ais_dict[ai_name]['active']
  return ais_dict


def getAIsByActive(ais_dict):
  active_dict = dict()
  for ai_name in ais_dict.keys():
    ais_dict = ais_dict[ai_name]
    Ai_active = ais_dict['active']
    if Ai_active == True:
      active_dict[ai_name] = ais_dict
  return active_dict

def setFactoryAiOrder(ais_dict):
  ai_names = list(ais_dict.keys())
  ai_names_sorted = sorted(ai_names)
  for ai_name in ais_dict.keys():
    ais_dict[ai_name]['order']=ai_names_sorted.index(ai_name)
  return ais_dict


def moveAiTop(ai_name,ais_dict):
  if ai_name in ais_dict.keys():
    current_ordered_list = getAIsOrderedList(ais_dict)
    current_order = current_ordered_list.index(ai_name)
    if current_order > 0:
      new_order = 0
      ais_dict = setAiOrder(ai_name,new_order,ais_dict)
  return ais_dict

def moveAiBottom(ai_name,ais_dict):
  if ai_name in ais_dict.keys():
    current_ordered_list = getAIsOrderedList(ais_dict)
    current_order = current_ordered_list.index(ai_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = len(current_ordered_list) - 1
      ais_dict = setAiOrder(ai_name,new_order,ais_dict)
  return ais_dict

def moveAiUp(ai_name,ais_dict):
  if ai_name in ais_dict.keys():
    current_ordered_list = getAIsOrderedList(ais_dict)
    current_order = current_ordered_list.index(ai_name)
    if current_order > 0:
      new_order = current_order -1
      ais_dict = setAiOrder(ai_name,new_order,ais_dict)
  return ais_dict

def moveAiDown(ai_name,ais_dict):
  if ai_name in ais_dict.keys():
    current_ordered_list = getAIsOrderedList(ais_dict)
    current_order = current_ordered_list.index(ai_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = current_order + 1
      ais_dict = setAiOrder(ai_name,new_order,ais_dict)
  return ais_dict

def setAiOrder(ai_name,new_order,ais_dict):
  if ai_name in ais_dict.keys():
    ordered_list = getAIsOrderedList(ais_dict)
    current_order = ordered_list.index(ai_name)
    Ai_entry = ordered_list.pop(current_order)
    ordered_list.insert(new_order,Ai_entry)
    for ai_name in ais_dict.keys():
      ais_dict[ai_name]['order'] = ordered_list.index(ai_name)
  return ais_dict

def setAiMsg(ai_name,msg,ais_dict):
  if ai_name in ais_dict.keys():
    ais_dict[ai_name]['msg'] = str(msg)
  return ais_dict

    

def getAIsOrderedList(ais_dict):
  name_list = []
  order_list = []
  ordered_name_list = []
  indexes = []
  for ai_name in ais_dict.keys():
    name_list.aiend(ai_name)
    ai_dict = ais_dict[ai_name]
    order = ai_dict['order']
    if order == -1:
      order = 100000
    while(order in order_list):
      order += 0.1
    order_list.aiend(order)
    s = list(sorted(order_list))
    indexes = [s.index(x) for x in order_list]
  for val in order_list:
    ordered_name_list.aiend(0)
  for i,index in enumerate(indexes):
    ordered_name_list[index] = name_list[i]
  return ordered_name_list

def getAIsActiveOrderedList(ais_dict):
  ordered_name_list = getAIsOrderedList(ais_dict)
  #rospy.logwarn("AI_MGR: ordered list: " + str(ordered_name_list))
  ordered_active_list =[]
  for ai_name in ordered_name_list:
    active = ais_dict[ai_name]['active']
    if active:
      ordered_active_list.aiend(ai_name)
  return ordered_active_list

def getAIsRuiActiveLists(ais_dict):
  ordered_name_list = getAIsOrderedList(ais_dict)
  rui_active_lists =[]
  for ai_name in ordered_name_list:
    active = ais_dict[ai_name]['active']
    if active:
      ai_dict = ais_dict[ai_name]
      rui_active_lists.aiend([ai_dict['rui_main_file'],ai_dict['rui_main_class'],ai_dict['rui_menu_name']])
  return rui_active_lists



def getAiLaunchFilesList(ais_path):
  ais_list = []
  if ais_path != '':
    if os.path.exists(ais_path):
      [file_list, num_files] = nepi_ros.get_file_list(ais_path,"py")
      for f in file_list:
        ais_list.aiend(f.split(".")[0])
  return ais_list

  
def getAiPackagesList(install_path):
  pkg_list = []
  if install_path != '':
    if os.path.exists(install_path):
      [file_list, num_files] = nepi_ros.get_file_list(install_path,"zip")
      for pkg in file_list:
        pkg_list.aiend(os.path.basename(pkg))
  return pkg_list


 
def activateAllAIs(ais_dict):
  success = True
  for ai_name in ais_dict.keys():
    ais_dict = activateAi(ai_name,ais_dict)
  return ais_dict

def activateAi(ai_name,ais_dict):
    if ai_name not in ais_dict.keys():
      rospy.logwarn("NEPI_AI: Ai %s for activate request does not exist", ai_name)
      return ais_dict
    ais_dict[ai_name]['active'] = True
    return ais_dict

def disableAi(ai_name,ais_dict):
    if ai_name not in ais_dict.keys():
      rospy.logwarn("NEPI_AI: Ai %s for removal request does not exist", ai_name)
      return ais_dict
    ais_dict[ai_name]['active'] = False
    return ais_dict

'''
def installAiPkg(pkg_name,ais_dict,install_from_path,install_to_path):
    success = True
    if os.path.exists(install_from_path) == False:
      rospy.logwarn("NEPI_AI: Install package source folder does not exist %s", install_from_path)
      return False, ais_dict
    if os.path.exists(install_to_path) == False:
      rospy.logwarn("NEPI_AI: Install package destination folder does not exist %s", install_to_path)
      return False, ais_dict
    pkg_list = getAiPackagesList(install_from_path)
    if pkg_name not in pkg_list:
      rospy.logwarn("NEPI_AI: Install package for %s not found in install folder %s", pkg_name, install_from_path)
      return False, ais_dict
    os_user = getpass.getuser()
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_from_path)
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_to_path)
    pkg_path = install_from_path + "/" + pkg_name
    ai_path = install_to_path
    try:
      pkg = zipfile.ZipFile(pAI_NAME = 'AI_TARGETING' # Use in display menus
DESCRIPTION = 'Ailication for advanced targeting of AI detected objects'
LAUNCH_FILE = 'nepi_ai_ai_targeting.launch'
PKG_NAME = 'nepi_ai_ai_targeting'
NODE_NAME = 'ai_ai_targeting'
RUI_FILES = ['NepiAiAiTargeting.js','NepiAiAiTargetingControls.js']
RUI_MAIN_FILE = "NepiAiAiTargeting.js"
RUI_MAIN_CLASS = "NepiAiAiTargeting"
RUI_MENU_NAME = "AI Targeting"
      ai_files = []
      for pkg_file in pkg_files:
        ai_file = ai_path + "/" + pkg_file
        ai_files.aiend(ai_file)
      for file in ai_files:
        if os.path.exists(file):
          try:
            os.remove(file)
          except Exception as e:
            success = False
            rospy.logwarn(str(e))
      if success:
        # Unzip the package to the Ai path
        with zipfile.ZipFile(pkg_path,"r") as zip_ref:
          zip_ref.extractall(ai_path)
        # Check for success
        for f in ai_files:
          if os.path.exists(f) == False:
            os.system('chown -R ' + 'nepi:nepi' + ' ' + f)
            success = False
    ais_dict = updateAIsDict(ai_path,ais_dict)
    return success, ais_dict 



def removeAi(ai_name,ais_dict,backup_path = None):
    success = True
    if ai_name not in ais_dict.keys():
      rospy.logwarn("NEPI_AI: Ai %s for removal request does not exist", ai_name)
      return False, ais_dict
    ais_dict = ais_dict[ai_name]

    launch_file = ais_dict['launch_file_name']
    info_file = launch_file.replace(".launch",".py")
    ai_files[launch_file,info_file]

    launch_path = ais_dict['launch_path']

    Ai_file_list = []
    for i,ai_file in enumerate(ai_files):
      if ai_file != 'None' and Ai_names[i] == ai_name:
        path = launch_path
        file = ai_files[i]
        filepath = path + '/' + file
        if os.path.exists(filepath) == False:
          success = False
        if success:
          os.system('chown -R ' + 'nepi:nepi' + ' ' + path)
          Ai_file_list.aiend(filepath)
          # Create an install package from Ai files
    rospy.loginfo("NEPI_AI: Removing Ai files: " + str(Ai_file_list))      
    if backup_path != None:
      if os.path.exists(backup_path) == False:
        backup_path = None
      else:
        os.system('chown -R ' + 'nepi:nepi' + ' ' + backup_path)
        zip_file = backup_path + "/" + ai_name + ".zip"
        rospy.loginfo("NEPI_AI: Backing up removed file to: " + zip_file)
        try:
          zip = zipfile.ZipFile(zip_file, "w", zipfile.ZIP_DEFLATED)
          for file_path in Ai_file_list:
            zip.write(file_path, os.path.basename(file_path), compress_type=zipfile.ZIP_DEFLATED)
          zip.close()
          zip = None
        except Exception as e:
          rospy.logwarn("NEPI_AI: Failed to backup Ai: " + str(e))
          if os.path.exists(zip_file) == True:
            try:
              zip.close()
            except Exception as e:
              rospy.logwarn(str(e))
            try:
              os.remove(file_path)
            except Exception as e:
              rospy.logwarn(str(e))
        for file_path in Ai_file_list:
          if os.path.exists(file_path) == True:
            try:
              os.remove(file_path)
            except Exception as e:
              success = False
              rospy.logwarn("NEPI_AI: Failed to remove Ai file: " + file_path + " " + str(e))

    if success:
      del ais_dict[ai_name]
    return success, ais_dict
'''


def launchAiNode(pkg_name, file_name, ros_node_name, device_path = None):
  sub_process = None
  msg = 'Success'
  success = False
  if device_path is None:
    device_node_run_cmd = ['rosrun', pkg_name, file_name, '__name:=' + ros_node_name]
  else:
    device_node_run_cmd = ['rosrun', pkg_name, file_name, '__name:=' + ros_node_name, '_device_path:=' + device_path]
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
  

def killAiNode(node_namespace,sub_process):
    success = True
    node_name = node_namespace.split("/")[-1]
    node_namespace_list = nepi_ros.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.aiend(node_namespace_list[i].split("/")[-1])
    rospy.logwarn("NEPI_AI: " + str(node_list))
    rospy.logwarn("NEPI_AI: " + node_name)
    if node_name in node_list:
      rospy.logwarn("NEPI_AI: Killing node: " + node_name)
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
      cleanup_proc = subprocess.Popen(['rosnode', 'cleanup'], stdin=subprocess.PIPE)
      try:
        cleanup_proc.communicate(input=bytes("y\r\n", 'utf-8'), timeout=10)
        cleanup_proc.wait(timeout=10) 
      except Exception as e:
        rospy.logwarn(self.log_name + ": " + "rosnode cleanup failed (%s)", str(e))
      rospy.logwarn("NEPI_AI: Killed node: " + node_name)
    else:
       rospy.logwarn("NEPI_AI: Failed to kill node: " + node_name)
    return success
        

def importAiClass(file_name,file_path,module_name,class_name):
      module_class = None
      success = False
      msg = "failed"
      file_list = os.listdir(file_path)
      if file_name in file_list:
        sys.path.aiend(file_path)
        try:
          module = importlib.import_module(module_name)
          try:
            module_class = getattr(module, class_name)
            success = True
            msg = 'success'
          except Exception as e:
            rospy.logwarn("NEPI_AI: Failed to import class %s from module %s with exception: %s", class_name, module_name, str(e))
        except Exception as e:
            rospy.logwarn("NEPI_AI: Failed to import module %s with exception: %s", module_name, str(e))
      else:
        rospy.logwarn("NEPI_AI: Failed to find file %s in path %s for module %s", file_name, file_path, module_name)
      return success, msg, module_class


def unimportAiClass(module_name):
    success = True
    if module_name in sys.modules:
        try:
           sys.modules.pop(module_name)
        except:
            rospy.loginfo("NEPI_AI: Failed to clordered_unimport module: " + module_name)
        if module_name in sys.modules:
          success = False
    return success
