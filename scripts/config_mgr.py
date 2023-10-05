#!/usr/bin/env python
#TODO License

# The config_mgr node serves as a bridge between the ROS param server and the filesystem.
# It provides the rest of the Numurus/ROS node set the ability to save and restore config.
# files with rudimentary coordination to reduce overloading file system during times of
# heavy updates.

import rospy
import rosparam

import os
import errno

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.srv import FileReset

CFG_PATH = '/opt/nepi/ros/etc/'
CFG_SUFFIX = '.yaml'
FACTORY_SUFFIX = '.num_factory'

USER_CFG_PATH = '/mnt/nepi_storage/user_cfg'
USER_SUFFIX = '.user'

pending_nodes = {}

# Moving symlinks is typically faster and more robust than copying files, so to reduce the
# chance of filesystem corruption in the event of e.g., power failure, we use a symlink-based config
# file scheme.
def symlink_force(target, link_name):
    try:
        os.symlink(target, link_name)
    except OSError as e:
        if e.errno == errno.EEXIST:
            os.remove(link_name)
            os.symlink(target, link_name)
        else:
            rospy.logerr("Unable to create symlink %s for %s", link_name, target)
            return False
    return True

def separate_node_name_in_msg(qualified_node_name):
    return qualified_node_name.split("/")[-1]

def update_from_file(file_pathname, namespace):
    try:
        paramlist = rosparam.load_file(file_pathname, namespace, verbose=True)
        for params, ns in paramlist:
            rosparam.upload_params(ns, params, verbose=True)
    except:
        rospy.logerr("Unable to load factory parameters from file %s", file_pathname)
        return [False]

    return [True]

def get_cfg_pathname(qualified_node_name):
    node_name = separate_node_name_in_msg(qualified_node_name)
    cfg_pathname = CFG_PATH + node_name  + '/' + node_name + CFG_SUFFIX
    return cfg_pathname

def get_user_cfg_pathname(qualified_node_name):
    node_name = separate_node_name_in_msg(qualified_node_name)
    user_cfg_pathname = USER_CFG_PATH + '/' + node_name + CFG_SUFFIX + USER_SUFFIX
    return user_cfg_pathname

def user_reset(req):
    qualified_node_name = req.node_name
    cfg_pathname = get_cfg_pathname(qualified_node_name)

    # Now update the param server
    return update_from_file(cfg_pathname, qualified_node_name)

def factory_reset(req):
    qualified_node_name = req.node_name
    cfg_pathname = get_cfg_pathname(qualified_node_name)
    factory_cfg_pathname = cfg_pathname + FACTORY_SUFFIX

    # First, move the symlink
    if False == symlink_force(factory_cfg_pathname, cfg_pathname):
        return [False] # Error logged upstream

    # Now update the param server
    return update_from_file(cfg_pathname, qualified_node_name)

def store_params(msg):
    qualified_node_name = msg.data
    user_cfg_pathname = get_user_cfg_pathname(qualified_node_name)
    
    # First, write to the user file
    rosparam.dump_params(user_cfg_pathname, qualified_node_name)

    # Now, ensure the link points to the correct file
    cfg_pathname = get_cfg_pathname(qualified_node_name)
    if (False == symlink_force(user_cfg_pathname, cfg_pathname)):
        rospy.logerr("Unable to update the cfg. file link")

def restore_user_cfgs(msg):
    for root, dirs, files in os.walk(CFG_PATH):
        for name in files:
            full_name = os.path.join(root, name)
            if full_name.endswith(CFG_SUFFIX) and os.path.islink(full_name):
                user_cfg_name = os.path.join(USER_CFG_PATH, name + USER_SUFFIX)
                if os.path.exists(user_cfg_name): # Restrict to those with present user configs
                    link_name = os.path.join(root, name.replace(FACTORY_SUFFIX, ''))
                    rospy.loginfo("Updating " + link_name + " to user config")
                    symlink_force(user_cfg_name, link_name)

def config_mgr():
    rospy.init_node('config_mgr')

    rospy.loginfo('Starting the config. mgr node')

    rospy.Subscriber('store_params', String, store_params)
    rospy.Subscriber('full_user_restore', Empty, restore_user_cfgs)

    rospy.Service('factory_reset', FileReset, factory_reset)
    rospy.Service('user_reset', FileReset, user_reset)

    rospy.spin()

if __name__ == '__main__':
    config_mgr()
