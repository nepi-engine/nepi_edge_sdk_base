#!/usr/bin/env python
# TODO License
import os
import os.path
from shutil import copyfile
import re
import errno

import rospy

from std_msgs.msg import String
from num_sdk_base.msg import FactoryReset
from num_sdk_base.srv import *

FACTORY_CFG_SUFFIX = '.num_factory'
USER_CFG_SUFFIX = '.user'

CHRONY_CFG_BASENAME = '/etc/chrony/chrony.conf'

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

def ensure_user_conf(basename_path):
	userconf_path = basename_path + USER_CFG_SUFFIX
	if (False == os.path.isfile(userconf_path)):
		# Need to create it from a copy of the factory config
		factoryconf_path = basename_path + FACTORY_CFG_SUFFIX
		try:
			copyfile(factoryconf_path, userconf_path)
		except:
			rospy.logerr("Unable to copy %s to %s", factoryconf_path, userconf_path)
			return False

	return symlink_force(userconf_path, basename_path)

def reset_to_factory_conf(basename_path):
	userconf_path = basename_path + USER_CFG_SUFFIX
	factoryconf_path = basename_path + FACTORY_CFG_SUFFIX

	symlink_force(factoryconf_path, basename_path)
	if (True == os.path.isfile(userconf_path)):
		os.remove(userconf_path)
		rospy.loginfo("Removed user config %s", userconf_path)

	# TODO: Restart the application?

def add_server(server_host):
	if (False == ensure_user_conf(CHRONY_CFG_BASENAME)):
		return

	userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX
	
	#ensure just a simple hostname is being added
	host = server_host.data.split()[0]

	new_server_cfg_line = 'server ' + host
	match_line = '^' + new_server_cfg_line
	file = open(userconf_path, 'r+')
	found_match = False
	for line in file.readlines():
		if re.search(match_line, line):
			rospy.loginfo("Ignoring redundant NTP server additions for %s", host)
			found_match = True
			break

	#At EOF, so just write here
	if (False == found_match):
		rospy.loginfo("Adding new NTP server %s", host)
		file.write(new_server_cfg_line + '\n')

	# TODO: Restart the application to take immediate effect?

def remove_server(server_host):
	userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX
	if (False == os.path.isfile(userconf_path)):
		rospy.loginfo("Ignoring request to remove NTP server since factory config is in use")
		return

	# Make sure the symlink points to the user config - we've already established that user cfg exists
	if (False == ensure_user_conf(CHRONY_CFG_BASENAME)):
		return

	#ensure just a simple hostname is being added
	host = server_host.data.split()[0]

	server_cfg_line = 'server ' + host
	match_line = '^server ' + host
	# Must copy the file line-by-line to a tmp, then overwrite the original
	orig_file = open(userconf_path, 'r')
	tmpfile_path = userconf_path + ".tmp"
	tmp_file = open(tmpfile_path, 'w')
	for line in orig_file.readlines():
		if re.search(match_line, line):
			# Don't write this line as we want to eliminate it
			rospy.loginfo("Removing NTP server %s", host)
			continue
		else:
			tmp_file.write(line)

	orig_file.close()
	tmp_file.close()
	os.rename(tmpfile_path, userconf_path)


def factory_reset(reset_id):
	if reset_id.id == FactoryReset.ID_NTP or reset_id == FactoryReset.ID_ALL:
		rospy.loginfo("Restoring NTP to factory config")
		reset_to_factory_conf(CHRONY_CFG_BASENAME)

def list_ntp_servers(req):
	resp = []
	file = open(CHRONY_CFG_BASENAME, 'r')
	for line in file.readlines():
		if re.search('^server', line):
			resp.append(line.split()[1])
	return ListNTPServersResponse(resp)

def time_sync_mgr():
	rospy.init_node('time_sync_mgr')

	rospy.loginfo("Starting the Time Sync Manager node")

	rospy.Subscriber('add_ntp_server', String, add_server)
	rospy.Subscriber('remove_ntp_server', String, remove_server)
	rospy.Subscriber('factory_reset', FactoryReset, factory_reset)

	rospy.Service('list_ntp_servers', ListNTPServers, list_ntp_servers)

	rospy.spin()

if __name__ == '__main__':
	time_sync_mgr()

