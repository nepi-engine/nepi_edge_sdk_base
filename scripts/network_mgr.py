#!/usr/bin/env python
# TODO License

import rospy
import socket
import subprocess
import collections
import os
from datetime import datetime

from std_msgs.msg import String, Bool, Empty, Int32
from nepi_ros_interfaces.msg import Reset
from nepi_ros_interfaces.srv import IPAddrQuery, FileReset, BandwidthUsageQuery, WifiQuery

class NetworkMgr:
    """The Network Manager Node of the Numurus core SDK.

    This node controls network IP settings. Users are not able to override the factory configuration
    but they can add and remove additional IPv4 addresses.
    """

    NODE_NAME = "network_mgr"
    NET_IFACE = "eth0"
    WONDERSHAPER_CALL = "/opt/nepi/ros/share/wondershaper/wondershaper"
    BANDWIDTH_MONITOR_PERIOD_S = 2.0

    FACTORY_STATIC_IP_FILE = "/opt/nepi/ros/etc/linux_cfg/num_static_ipv4"
    USER_STATIC_IP_FILE = "/opt/nepi/ros/etc/network_mgr/numurus_user_ip_aliases"
    USER_STATIC_IP_FILE_PREFACE = "# This file includes all user-added IP address aliases. It is sourced by the top-level static IP addr file.\n\n"

    # Following are to support changing rosmaster IP address
    SYS_ENV_FILE = "/opt/nepi/sys_env.bash"
    ROS_MASTER_PORT = 11311
    ROSLAUNCH_FILE = "/opt/nepi/ros/etc/roslaunch.sh"
    REMOTE_ROS_NODE_ENV_LOADER_FILES = ["numurus@num-sb1-zynq:/opt/nepi/ros/etc/env_loader.sh"]

    # Following support WiFi setup
    WIFI_IFACE = "wlan0"
    CREATE_AP_CALL = "/opt/nepi/ros/share/create_ap/create_ap"
    DEFAULT_WIFI_AP_NAME = "nepi_device_ap"
    DEFAULT_WIFI_AP_PASSWORD = "nepi_device_ap"

    store_params_publisher = None

    def run(self):
        rospy.spin()

    def cleanup(self):
        self.process.stop()

    def get_primary_ip_addr(self):
        key = "inet static"
        with open(self.FACTORY_STATIC_IP_FILE, "r") as f:
            lines = f.readlines()
            for i,line in enumerate(lines):
                if key in line:
                    primary_ip = lines[i+1].split()[1]
                    return primary_ip
        return "Unknown Primary IP"

    def get_current_ip_addrs(self):
        primary_ip = self.get_primary_ip_addr()
        ip_addrs = [primary_ip]
        addr_list_output = subprocess.check_output(['ip','addr','list','dev',self.NET_IFACE])
        tokens = addr_list_output.split()
        for i, t in enumerate(tokens):
            if (t == 'inet'):
                # Ensure that aliases go at the back of the list and primary remains at the front -- we rely on that ordering throughout this module
                if (tokens[i+1] != primary_ip):
                    ip_addrs.append(tokens[i+1]) # Back of the line
        return ip_addrs

    def validate_cidr_ip(self, addr):
        # First, validate the input
        tokens = addr.split('/')
        new_ip = tokens[0]
        new_ip_bits = 0
        try:
            new_ip_bits = socket.inet_aton(new_ip)
        except:
            rospy.logerr("Rejecting invalid IP address %s", new_ip)
            return False
        if (len(tokens) != 2):
            rospy.logerr("Rejecting invalid address must be in CIDR notation (x.x.x.x/y). Got %s", addr)
            return False
        cidr_netmask = (int)(tokens[1])
        if cidr_netmask < 1 or cidr_netmask > 32:
            rospy.logerr("Rejecting invalid CIDR netmask (got %s)", addr)
            return False

        # Finally, verify that this isn't the "fixed" address on the device. Don't let anyone sneak past the same
        # address in a different numerical format - compare as a 32-bit val
        fixed_ip_addr = self.get_current_ip_addrs()[0].split('/')[0]
        fixed_ip_bits = 0
        try:
            fixed_ip_bits = socket.inet_aton(fixed_ip_addr)
        except:
            rospy.logerr("Cannot validate IP address becaused fixed IP %s appears invalid", fixed_ip_addr)
            return False
        if (new_ip_bits == fixed_ip_bits):
            rospy.logerr("IP address invalid because it matches fixed primary IP")
            return False

        return True

    def add_ip_impl(self, new_addr):
        try:
            subprocess.check_call(['ip','addr','add',new_addr,'dev',self.NET_IFACE])
        except:
            rospy.logerr("Failed to set IP address to %s", new_addr)
    def add_ip(self, new_addr_msg):
        if True == self.validate_cidr_ip(new_addr_msg.data):
            self.add_ip_impl(new_addr_msg.data)
        else:
            rospy.logerr("Unable to add invalid/ineligible IP address")

    def remove_ip_impl(self, old_addr):
        try:
            subprocess.check_call(['ip','addr','del',old_addr,'dev',self.NET_IFACE])
        except:
            rospy.logerr("Failed to remove IP address %s", old_addr)

    def remove_ip(self, old_addr_msg):
        if True == self.validate_cidr_ip(old_addr_msg.data):
            self.remove_ip_impl(old_addr_msg.data)
        else:
            rospy.logerr("Unable to remove invalid/ineligible IP address")

    def enable_dhcp_impl(self, enabled):
        if enabled is True:
            if self.dhcp_enabled is False:
                rospy.loginfo("Enabling DHCP Client")
                try:
                    subprocess.check_call(['dhclient', '-nw', self.NET_IFACE])
                    self.dhcp_enabled = True
                except Exception as e:
                    rospy.logerr("Unable to enable DHCP: " + str(e))
            else:
                rospy.loginfo("DHCP already enabled")
        else:
            if self.dhcp_enabled is True:
                rospy.loginfo("Disabling DHCP Client")
                try:
                    # The dhclient -r call below causes all IP addresses on the interface to be dropped, so
                    # we reinitialize them here... this will not work for IP addresses that were
                    # added in this session but not saved to config (i.e., not known to param server)

                    subprocess.check_call(['dhclient', '-r', self.NET_IFACE])
                    self.dhcp_enabled = False
                    rospy.sleep(1)

                    # Restart the interface -- this picks the original static IP back up and sources the user IP alias file
                    subprocess.call(['ifdown', self.NET_IFACE])
                    rospy.sleep(1)
                    subprocess.call(['ifup', self.NET_IFACE])

                except Exception as e:
                    rospy.logerr("Unable to disable DHCP: " + str(e))
            else:
                rospy.loginfo("DHCP already disabled")

    def enable_dhcp(self, enabled_msg):
        self.enable_dhcp_impl(enabled_msg.data)

    def set_dhcp_from_params(self):
        if (rospy.has_param('~dhcp_enabled')):
            enabled = rospy.get_param('~dhcp_enabled')
            if self.dhcp_enabled != enabled:
                self.enable_dhcp_impl(enabled)

    def reset(self, msg):
        if Reset.USER_RESET == msg.reset_type:
            user_reset_proxy = rospy.ServiceProxy('user_reset', FileReset)
            try:
                resp = user_reset_proxy(self.NODE_NAME)
            except rospy.ServiceException as exc:
                rospy.logerr("{}: unable to execute user reset".format(self.NODE_NAME))
                return
            self.set_dhcp_from_params()
        elif Reset.FACTORY_RESET == msg.reset_type:
            factory_reset_proxy = rospy.ServiceProxy('factory_reset', FileReset)
            try:
                resp = factory_reset_proxy(self.NODE_NAME)
            except rospy.ServiceException as exc:
                rospy.logerr("{}: unable to execute factory reset".format(self.NODE_NAME))
                return

            # Overwrite the user static IP file with the blank version
            with open(self.USER_STATIC_IP_FILE, "w") as f:
                f.write(self.USER_STATIC_IP_FILE_PREFACE)

            # Set the rosmaster back to localhost
            self.set_rosmaster_impl("localhost")

            self.set_dhcp_from_params()
            rospy.logwarn("{}: Factory reset complete -- must reboot device for IP and ROS_MASTER_URI changes to take effect")

        elif Reset.SOFTWARE_RESET:
            rospy.signal_shutdown("{}: shutdown by request".format(self.NODE_NAME))
        elif Reset.HARDWARE_RESET:
            rospy.loginfo("{}: Executing hardware reset by request".format(self.NODE_NAME))
            # Reset the interface
            subprocess.call(['ifdown', self.NET_IFACE])
            rospy.sleep(1)
            subprocess.call(['ifup', self.NET_IFACE])
        else:
            rospy.logerr("{}: invalid reset value (%u)", msg.reset_type)

    def save_config(self, msg):
        # First update user static IP file
        # Note that this is outside the scope of ROS param server because we need these
        # aliases to come up even before ROS (hence this node) comes up in case the remoted ROSMASTER
        # is on a subnet only reachable via one of these aliases
        current_ips = self.get_current_ip_addrs()

        if (len(current_ips) > 1):
            with open(self.USER_STATIC_IP_FILE, "w") as f:
                f.write(self.USER_STATIC_IP_FILE_PREFACE)
                for i,ip_cidr in enumerate(current_ips[1:]): # Skip the first one -- that is the factory default
                    alias_name = self.NET_IFACE + ":" + str(i+1)
                    f.write("auto " + alias_name + "\n")
                    f.write("iface " + alias_name + " inet static\n")
                    f.write("    address " + ip_cidr + "\n\n")

        # DHCP Settings are stored in the ROS config file
        rospy.set_param('~dhcp_enabled', self.dhcp_enabled)

        # Wifi settings are stored in the ROS config file
        rospy.set_param('~wifi/enable_access_point', self.wifi_ap_enabled)
        rospy.set_param('~wifi/access_point_name', self.wifi_ap_name)
        rospy.set_param('~wifi/access_point_password', self.wifi_ap_password)

        self.store_params_publisher.publish(rospy.get_name())

    def set_upload_bwlimit(self, msg):
        if msg.data >= 0 and msg.data < 1:
            rospy.logerr('Cannot set bandwidth limit below 1Mbps')
            return

        # First, update param server
        rospy.set_param('~tx_bw_limit_mbps', msg.data)

        # Now set from value from param server
        self.set_upload_bw_limit_from_params()

    def set_rosmaster(self, msg):
        new_master_ip = msg.data
        self.set_rosmaster_impl(new_master_ip)

    def set_rosmaster_impl(self, master_ip):
        auto_comment = " # Modified by network_mgr " + str(datetime.now()) + "\n"

        # First, determine if the master is local, either as localhost or one of the configured IP addrs
        master_is_local = True if (master_ip == "localhost") else False
        if master_is_local is False:
            local_ips = self.get_current_ip_addrs()
            for ip_cidr in local_ips:
                ip = ip_cidr.split('/')[0]
                if master_ip == ip:
                    master_is_local = True
                    break

        if master_is_local is True:
            master_ip = "localhost" # Force 'localhost' whenever the "new" master is a local IP in case that local IP (alias) is later removed
            master_ip_for_remote_hosts = self.get_primary_ip_addr().split('/')[0]
        else:
            master_ip_for_remote_hosts = master_ip
            # Now ensure we can contact the new rosmaster -- if not, bail out
            ret_code = subprocess.call(['nc', '-zvw5', master_ip,  str(self.ROS_MASTER_PORT)])
            if (ret_code != 0):
                rospy.logerr("Failed to detect a remote rosmaster at " + master_ip + ":" + str(self.ROS_MASTER_PORT) + "... refusing to update ROS_MASTER_URI")
                return

        # Edit the sys_env file appropriately
        rosmaster_line_prefix = "export ROS_MASTER_URI="
        new_rosmaster_line = rosmaster_line_prefix + "http://" + master_ip + ":" + str(self.ROS_MASTER_PORT) + auto_comment
        sys_env_output_lines = []
        with open(self.SYS_ENV_FILE, "r") as f_in:
            for line in f_in:
                if rosmaster_line_prefix in line:
                    line = new_rosmaster_line
                sys_env_output_lines.append(line)
        with open(self.SYS_ENV_FILE, "w") as f_out:
            f_out.writelines(sys_env_output_lines)

        # And the roslaunch file (add --wait for remote ros master)
        roslaunch_line_prefix = "roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}"
        new_roslaunch_line = roslaunch_line_prefix
        if master_is_local is False:
            new_roslaunch_line += " --wait"
        new_roslaunch_line += auto_comment
        roslaunch_output_lines = []
        with open(self.ROSLAUNCH_FILE, "r") as f_in:
            for line in f_in:
                if roslaunch_line_prefix in line:
                    line = new_roslaunch_line
                roslaunch_output_lines.append(line)
        with open(self.ROSLAUNCH_FILE, "w") as f_out:
            f_out.writelines(roslaunch_output_lines)

        # And the env_loader files for remote machines
        new_rosmaster_line = rosmaster_line_prefix + "http://" + master_ip_for_remote_hosts + ":" + str(self.ROS_MASTER_PORT) + auto_comment
        tmp_env_loader_file = "./env_loader_file.tmp"
        for remote_env_loader_file in self.REMOTE_ROS_NODE_ENV_LOADER_FILES:
            env_loader_lines = []
            ret_code = subprocess.call(['scp', remote_env_loader_file, tmp_env_loader_file])
            if (ret_code != 0):
                rospy.logwarn("Failed to get copy of remote file " + remote_env_loader_file + "... not updating ROS_MASTER_URI for that remote host")
                continue
            with open(tmp_env_loader_file, "r") as f_in:
                for line in f_in:
                    if rosmaster_line_prefix in line:
                        line = new_rosmaster_line
                    env_loader_lines.append(line)
            with open(tmp_env_loader_file, "w") as f_out:
                f_out.writelines(env_loader_lines)
            ret_code = subprocess.call(['scp', tmp_env_loader_file, remote_env_loader_file])
            if (ret_code != 0):
                rospy.logwarn("Failed to update remote file " + remote_env_loader_file)
            os.remove(tmp_env_loader_file)

        rospy.logwarn("Updated ROS_MASTER_URI to " + master_ip + "... requires reboot to complete the switch")

    def set_upload_bw_limit_from_params(self):
        bw_limit_mbps = -1
        if (rospy.has_param('~tx_bw_limit_mbps')):
            bw_limit_mbps = rospy.get_param('~tx_bw_limit_mbps')
        else:
            rospy.logwarn("No tx_bw_limit_mbps param set... will clear all bandwidth limits")

        # Always clear the current settings
        try:
            subprocess.call([self.WONDERSHAPER_CALL, '-a', self.NET_IFACE, '-c'])
        except Exception as e:
            rospy.logerr("Unable to clear current bandwidth limits: " + str(e))
            return

        if bw_limit_mbps < 0: #Sentinel values to clear limits
            rospy.loginfo("Cleared bandwidth limits")
            return

        # Now acquire the param from param server and update
        bw_limit_kbps = bw_limit_mbps * 1000
        try:
            subprocess.call([self.WONDERSHAPER_CALL, '-a', self.NET_IFACE, '-u', str(bw_limit_kbps)])
            rospy.loginfo("Updated TX bandwidth limit to " + str(bw_limit_mbps) + " Mbps")
            #self.tx_byte_cnt_deque.clear()
        except Exception as e:
            rospy.logerr("Unable to set upload bandwidth limit: " + str(e))

    def enable_wifi_ap_handler(self, enabled_msg):
        if self.has_wifi is False:
            rospy.logwarn("Cannot enable WiFi access point - system has no WiFi")
            return
        
        # Just set the param and let the ...from_params() function handle the rest
        rospy.set_param("~wifi/enable_access_point", enabled_msg.data)
        self.set_wifi_from_params()

    def set_wifi_from_params(self):
        self.wifi_ap_enabled = rospy.get_param('~wifi/enable_access_point', False)
        self.wifi_ap_name = rospy.get_param('~wifi/access_point_name', self.DEFAULT_WIFI_AP_NAME)
        self.wifi_ap_password = rospy.get_param('~wifi/access_point_password', self.DEFAULT_WIFI_AP_PASSWORD)
        
        if self.wifi_ap_enabled is True:
            if self.has_wifi is False:
                rospy.logwarn("Cannot enable WiFi access point - system has no WiFi")
                return
            try:
                # Use the create_ap command line
                subprocess.check_call([self.CREATE_AP_CALL, '-n', '--redirect-to-localhost', '--isolate-clients', '--daemon',
                                       self.WIFI_IFACE, self.wifi_ap_name, self.wifi_ap_password])
                rospy.loginfo("Started WiFi access point: " + self.wifi_ap_name)
            except Exception as e:
                rospy.logerr("Unable to start wifi access point with " + str(e))
        else:
            try:
                subprocess.check_call([self.CREATE_AP_CALL, '--stop', self.WIFI_IFACE])
            except Exception as e:
                rospy.logwarn("Unable to terminate wifi access point: " + str(e))

        # TODO: Connect to external network if so configured. Because we don't typically run
        # NetworkManager Linux util, the nmcli schemes for accomplishing this do not work. Instead, we must
        # use wpa_supplicant. Can use wpa_passphrase to generate the input to wpa_supplicant as per
        # https://wiki.archlinux.org/title/wpa_supplicant#Connecting_with_wpa_passphrase

    def monitor_bandwidth_usage(self, event):
        with open('/sys/class/net/' + self.NET_IFACE + '/statistics/tx_bytes', 'r') as f:
            tx_bytes = int(f.read())
            self.tx_byte_cnt_deque.append(tx_bytes)
        with open('/sys/class/net/' + self.NET_IFACE + '/statistics/rx_bytes', 'r') as f:
            rx_bytes = int(f.read())
            self.rx_byte_cnt_deque.append(rx_bytes)

    def handle_ip_addr_query(self, req):
        ips = self.get_current_ip_addrs()
        return {'ip_addrs':ips, 'dhcp_enabled': self.dhcp_enabled}

    def handle_bandwidth_usage_query(self, req):
        tx_rate_mbps = 0
        if len(self.tx_byte_cnt_deque) > 1:
            tx_rate_mbps =  8 * (self.tx_byte_cnt_deque[1] - self.tx_byte_cnt_deque[0]) / (self.BANDWIDTH_MONITOR_PERIOD_S * 1000000)

        rx_rate_mbps = 0
        if len(self.rx_byte_cnt_deque) > 1:
            rx_rate_mbps = 8 * (self.rx_byte_cnt_deque[1] - self.rx_byte_cnt_deque[0]) / (self.BANDWIDTH_MONITOR_PERIOD_S * 1000000)

        tx_rate_limit_mbps = -1.0
        if (rospy.has_param('~tx_bw_limit_mbps')):
            tx_rate_limit_mbps = rospy.get_param('~tx_bw_limit_mbps')

        return {'tx_rate_mbps':tx_rate_mbps, 'rx_rate_mbps':rx_rate_mbps, 'tx_limit_mbps': tx_rate_limit_mbps}

    def handle_wifi_query(self, req):
        return {'has_wifi': self.has_wifi, 'wifi_ap_enabled': self.wifi_ap_enabled, 
                'wifi_ap_name': self.wifi_ap_name, 'wifi_ap_password': self.wifi_ap_password}

    def isWifiPresent(self):
        # Just check for the existence of the interface. Maybe more sophisticated in the future
        try:
            subprocess.check_call(['ip','addr','list','dev',self.WIFI_IFACE])
            return True
        except:
            return False

    def __init__(self):
        rospy.init_node(self.NODE_NAME)

        rospy.loginfo("Starting the {} node".format(self.NODE_NAME))

        self.dhcp_enabled = False # initialize to false -- will be updated in set_dhcp_from_params
        self.tx_byte_cnt_deque = collections.deque(maxlen=2)
        self.rx_byte_cnt_deque = collections.deque(maxlen=2)
        
        self.has_wifi = self.isWifiPresent()
        if self.has_wifi is True:
            rospy.loginfo("Detected WiFi (interface queried = " + self.WIFI_IFACE + ")")
            self.wifi_ap_enabled = False
            self.wifi_ap_name = self.DEFAULT_WIFI_AP_NAME
            self.wifi_ap_password = self.DEFAULT_WIFI_AP_PASSWORD

        # Initialize from the config file (which should be loaded ahead of this call)
        self.set_dhcp_from_params()

        self.set_upload_bw_limit_from_params()

        # Public namespace stuff
        rospy.Subscriber('add_ip_addr', String, self.add_ip)
        rospy.Subscriber('remove_ip_addr', String, self.remove_ip)
        rospy.Subscriber('enable_dhcp', Bool, self.enable_dhcp)
        rospy.Subscriber('reset', Reset, self.reset)
        rospy.Subscriber('save_config', Empty, self.save_config)
        rospy.Subscriber('set_tx_bw_limit_mbps', Int32, self.set_upload_bwlimit)
        rospy.Subscriber('set_rosmaster', String, self.set_rosmaster)

        # Private namespace stuff
        rospy.Subscriber('~reset', Reset, self.reset)
        rospy.Subscriber('~save_config', Empty, self.save_config)

        rospy.Service('ip_addr_query', IPAddrQuery, self.handle_ip_addr_query)
        rospy.Service('bandwidth_usage_query', BandwidthUsageQuery, self.handle_bandwidth_usage_query)
        rospy.Service('wifi_query', WifiQuery, self.handle_wifi_query)

        rospy.Timer(rospy.Duration(self.BANDWIDTH_MONITOR_PERIOD_S), self.monitor_bandwidth_usage)

        self.store_params_publisher = rospy.Publisher('store_params', String, queue_size=1)

        # Wifi stuff -- only enabled if WiFi is present
        self.wifi_ap_enabled = False
        self.wifi_ap_name = "n/a"
        self.wifi_ap_password = "n/a"
        
        self.has_wifi = self.isWifiPresent()
        if self.has_wifi is True:
            rospy.loginfo("Detected WiFi on " + self.WIFI_IFACE)
            self.set_wifi_from_params()

            rospy.Subscriber('enable_wifi_access_point', Bool, self.enable_wifi_ap_handler)
        else:
            rospy.loginfo("No WiFi detected (interface queried = " + self.WIFI_IFACE + ")")
                
        self.run()

if __name__ == "__main__":
    node = NetworkMgr()
