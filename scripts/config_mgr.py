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

from std_msgs.msg import String
from num_sdk_base.srv import FactoryReset

CFG_SUFFIX = '.yaml'
FACTORY_SUFFIX = '.num_factory'
USER_SUFFIX = '.user'

CFG_PATH = '/opt/numurus/ros/etc/'

BASE_NAMESPACE = '/numurus/'

pending_nodes = {}

# Moving symlinks is typically faster and more robust than copying files, so to reduce the 
# chance of filesystem corruption in the event of e.g., power failure, we use a symlink-based config
# file scheme.
def symlink_force(target, link_name):
	try:
		os.symlink(target, link_name)
	except OSError, e:
		if e.errno == errno.EEXIST:
			os.remove(link_name)
			os.symlink(target, link_name)
		else:
			rospy.logerr("Unable to create symlink %s for %s", link_name, target)
			return False
	return True

def factory_reset(req):
	node_name = req.node_name
	cfg_pathname = CFG_PATH + node_name + CFG_SUFFIX
	factory_cfg_pathname = cfg_pathname + FACTORY_SUFFIX
	namespace = BASE_NAMESPACE + node_name

	# First, move the symlink
	if False == symlink_force(factory_cfg_pathname, cfg_pathname):
		return [False] # Error logged upstream

	# Now update the param server
	try:
		paramlist = rosparam.load_file(cfg_pathname, namespace, verbose=True)
		for params, ns in paramlist:
			rosparam.upload_params(ns, params, verbose=True)
	except:
		rospy.logerr("Unable to load factory parameters from file %s", cfg_pathname)
		return [False]

	return [True]

def store_params(node_name):
	n = node_name.data
	cfg_pathname = CFG_PATH + n + CFG_SUFFIX
	user_cfg_pathname = cfg_pathname + USER_SUFFIX

	# First, write to the user file
	namespace = BASE_NAMESPACE + n
	rosparam.dump_params(user_cfg_pathname, namespace)

	# Now, ensure the link points to the correct file
	if (False == symlink_force(user_cfg_pathname, cfg_pathname)):
		rospy.logerr("Unable to update the cfg. file link")
	
def config_mgr():
	rospy.init_node('config_mgr')

	rospy.loginfo('Starting the config. mgr node')

	rospy.Subscriber('store_params', String, store_params)
	rospy.Service('factory_reset', FactoryReset, factory_reset)

	rospy.spin()

if __name__ == '__main__':
	config_mgr()