#!/usr/bin/env python
# TODO License
import os
import os.path
from shutil import copyfile
import re
import errno
import subprocess
import sys

import rospy

from std_msgs.msg import String
from std_msgs.msg import Time

from num_sdk_msgs.msg import Reset
from num_sdk_msgs.msg import TimeStatus

from num_sdk_msgs.srv import TimeStatusQuery, TimeStatusQueryResponse

FACTORY_CFG_SUFFIX = '.num_factory'
USER_CFG_SUFFIX = '.user'

CHRONY_CFG_BASENAME = '/etc/chrony/chrony.conf'
CHRONY_SYSTEMD_SERVICE_NAME = 'chrony.service'

g_last_set_time = rospy.Time(0.0)

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

def restart_systemd_service(service_name):
    subprocess.call(["systemctl", "restart", service_name])

def reset_to_factory_conf(basename_path):
    userconf_path = basename_path + USER_CFG_SUFFIX
    factoryconf_path = basename_path + FACTORY_CFG_SUFFIX

    symlink_force(factoryconf_path, basename_path)
    if (True == os.path.isfile(userconf_path)):
        os.remove(userconf_path)
        rospy.loginfo("Removed user config %s", userconf_path)

    # Restart chrony to allow changes to take effect
    restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)

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
        # Restart chrony to allow changes to take effect
        restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)

def remove_server(server_host):
    userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX
    if (False == os.path.isfile(userconf_path)):
        rospy.loginfo("Ignoring request to remove NTP server since factory config is in use")
        return

    # Make sure the symlink points to the user config  we've already established that user cfg exists
    if (False == ensure_user_conf(CHRONY_CFG_BASENAME)):
        return

    #ensure just a simple hostname is being added
    host = server_host.data.split()[0]

    server_cfg_line = 'server ' + host
    match_line = '^server ' + host
    # Must copy the file linebyline to a tmp, then overwrite the original
    orig_file = open(userconf_path, 'r')
    tmpfile_path = userconf_path + ".tmp"
    tmp_file = open(tmpfile_path, 'w')
    found_it = False
    for line in orig_file.readlines():
        if re.search(match_line, line):
            # Don't write this line as we want to eliminate it
            rospy.loginfo("Removing NTP server %s", host)
            found_it = True
            continue
        else:
            tmp_file.write(line)

    orig_file.close()
    tmp_file.close()
    os.rename(tmpfile_path, userconf_path)

    if True == found_it:
        # Restart chrony to allow changes to take effect
        restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)

def reset(msg):
    if Reset.USER_RESET == msg.reset_type:
        # Nothing to do for a User Reset as config file is always up-to-date
        rospy.loginfo("Ignoring NTP user-reset NO-OP")
    elif Reset.FACTORY_RESET == msg.reset_type:
        rospy.loginfo("Restoring NTP to factory config")
        reset_to_factory_conf(CHRONY_CFG_BASENAME)
    elif Reset.SOFTWARE_RESET == msg.reset_type:
        rospy.loginfo("Executing soft reset for NTP")
        restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)
        rospy.signal_shutdown('Shutdown by request')
    elif Reset.HARDWARE_RESET == msg.reset_type:
        rospy.loginfo("Executing hard reset for NTP")
        # TODO: Any hardware restart required?
        restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)
        rospy.signal_shutdown('Shutdown by request')

def handle_time_status_query(req):
    time_status = TimeStatus()
    time_status.current_time = rospy.get_rostime()

    # Get Last PPS time from the sysfs node
    try:
        pps_string = subprocess.check_output(["cat", "/sys/class/pps/pps0/assert"])
        pps_tokens = pps_string.split('#')
        if (len(pps_tokens) >= 2):
            time_status.last_pps = rospy.Time(float(pps_string.split('#')[0]))
        else:
            time_status.last_pps = rospy.Time(0)
            rospy.logwarn_throttle(60, "Unable to parse /sys/class/pps/pps0/assert");
    except: # Failed to find the assert file - just return no PPS
        time_status.last_pps = rospy.Time(0)
        rospy.logwarn_throttle(60, "Unable to parse /sys/class/pps/pps0/assert");

        
    # Gather NTP info from chronyc application
    chronyc_sources = subprocess.check_output(["chronyc", "sources"]).splitlines()
    for line in chronyc_sources[1:]:
        if re.search('^\^|#', line): # Find sources lines by their leading "Mode" indicator
            tokens = line.split()
            time_status.ntp_sources.append(tokens[1]) # The source name is right after the first space
            time_status.last_ntp_sync.append(tokens[5])
            time_status.current_offset.append(tokens[6].split('[')[0]) # The string has two parts

    # Last set time (cheater clock sync method)
    time_status.last_set_time = g_last_set_time

    return  { 'time_status': time_status }

def set_time(msg):
    # TODO: Bounds checking?
    # Use the Linux 'date -s' command-line utility.
    rospy.loginfo("Setting time from set_time topic to %ds, %dns", msg.data.secs, msg.data.nsecs)
    timestring = '@' + str(float(msg.data.secs) + (float(msg.data.nsecs) / float(1e9))) 
    subprocess.call(["date", "-s", timestring])
    global g_last_set_time
    g_last_set_time = msg.data
    new_date = subprocess.check_output(["date"])
    rospy.loginfo("Updated date: %s", new_date)

    # TODO: Should we use this CTypes call into librt instead?
#    import ctypes
#    import ctypes.util
#    import time

    # /usr/include/linux/time.h:
    #
    # define CLOCK_REALTIME             0
#    CLOCK_REALTIME = 0

    # /usr/include/time.h
    #
    # struct timespec
    #  {
    #    __time_t tv_sec;        /* Seconds.  */
    #    long int tv_nsec;       /* Nanoseconds.  */
    #  };
    # Structure for the timespec arg. to librt::clock_settime()
#    class timespec(ctypes.Structure):
#        _fields_ = [("tv_sec", ctypes.c_long),
#                    ("tv_nsec", ctypes.c_long)]

    # Bring in the C-language librt
#    librt = ctypes.CDLL(ctypes.util.find_library("rt"))

    # Create a new timespec object
#    ts = timespec()
#    ts.tv_sec = int(msg.data.secs)
#    ts.tv_nsec = int(msg.data.nsecs)
#    rospy.logerr(ts.tv_sec)
#    rospy.logerr(ts.tv_nsec)

    # Now call into librt::clock_settime
#    rospy.logerr(librt.clock_settime(CLOCK_REALTIME, ctypes.byref(ts)))
#    rospy.logerr(os.strerror(ctypes.get_errno()))

def time_sync_mgr():
    rospy.init_node('time_sync_mgr', disable_signals=True)

    rospy.loginfo("Starting the Time Sync Manager node")

    # Public namespace stuff
    rospy.Subscriber('add_ntp_server', String, add_server)
    rospy.Subscriber('remove_ntp_server', String, remove_server)
    rospy.Subscriber('reset', Reset, reset)
    rospy.Subscriber('set_time', Time, set_time)

    # Private namespace stuff
    rospy.Subscriber('~reset', Reset, reset)

    rospy.Service('time_status_query', TimeStatusQuery, handle_time_status_query)

    rospy.spin()

if __name__ == '__main__':
    time_sync_mgr()
