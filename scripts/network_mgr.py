#!/usr/bin/env python
# TODO License

import rospy
import socket
import subprocess
import collections

from std_msgs.msg import String, Bool, Empty, Int32
from num_sdk_msgs.msg import Reset
from num_sdk_msgs.srv import IPAddrQuery, FileReset, BandwidthUsageQuery

class NetworkMgr:
    """The Network Manager Node of the Numurus core SDK.

    This node controls network IP settings. Users are not able to override the factory configuration
    but they can add and remove additional IPv4 addresses.
    """

    NODE_NAME = "network_mgr"
    NET_IFACE = "eth0"
    WONDERSHAPER_CALL = "/opt/numurus/ros/share/wondershaper/wondershaper"
    BANDWIDTH_MONITOR_PERIOD_S = 2.0

    store_params_publisher = None

    def run(self):
        rospy.spin()

    def cleanup(self):
        self.process.stop()

    def get_current_ip_addrs(self):
        ip_addrs = []
        addr_list_output = subprocess.check_output(['ip','addr','list','dev',self.NET_IFACE])
        tokens = addr_list_output.split()
        for i, t in enumerate(tokens):
            if (t == 'inet'):
                ip_addrs.append(tokens[i+1])
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
                try:
                    # The dhclient -r call below causes all IP addresses on the interface to be dropped, so
                    # we reinitialize them here... this will not work for IP addresses that were
                    # added in this session but not saved to config (i.e., not known to param server)

                    subprocess.check_call(['dhclient', '-r', self.NET_IFACE])
                    self.dhcp_enabled = False
                    rospy.sleep(1)

                    # Restart the interface -- this picks the original static IP back up
                    subprocess.call(['ifdown', self.NET_IFACE])
                    rospy.sleep(1)
                    subprocess.call(['ifup', self.NET_IFACE])

                    # Reset IPs from param server, but make sure to avoid the DHCP part because that param hasn't been updated at this point
                    self.set_ips_from_params(skip_dhcp = True)
                except Exception as e:
                    rospy.logerr("Unable to disable DHCP: " + str(e))
            else:
                rospy.loginfo("DHCP already disabled")

    def enable_dhcp(self, enabled_msg):
        self.enable_dhcp_impl(enabled_msg.data)

    def set_ips_from_params(self, skip_dhcp = False):
        curr_ips = self.get_current_ip_addrs()
        for ip in curr_ips[1:]: # Skip the first one -- that is the factory default
            rospy.loginfo("Purging IP address %s to produce a clean slate", ip)
            self.remove_ip_impl(ip)

        if (rospy.has_param('~configured_ip_addrs')):
            configured_ips = rospy.get_param('~configured_ip_addrs')
            for ip in configured_ips:
                rospy.loginfo("Adding configured IP address %s", ip)
                self.add_ip_impl(ip)

        if (skip_dhcp is False):
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
            self.set_ips_from_params()
            rospy.loginfo("{}: ignoring user reset because this node's config is always up-to-date".format(self.NODE_NAME))
        elif Reset.FACTORY_RESET == msg.reset_type:
            factory_reset_proxy = rospy.ServiceProxy('factory_reset', FileReset)
            try:
                resp = factory_reset_proxy(self.NODE_NAME)
            except rospy.ServiceException as exc:
                rospy.logerr("{}: unable to execute factory reset".format(self.NODE_NAME))
                return
            self.set_ips_from_params()
        elif Reset.USER_RESET:
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
        # First update param server - only set entries 1:end because 0 is
        # the fixed IP
        current_ips = self.get_current_ip_addrs()
        if (len(current_ips) > 1):
            rospy.set_param('~configured_ip_addrs', current_ips[1:])
        else:
            rospy.set_param('~configured_ip_addrs', [])

        rospy.set_param('~dhcp_enabled', self.dhcp_enabled)

        # Now tell config_mgr to save the file
        self.store_params_publisher.publish(rospy.get_name())

    def set_upload_bwlimit(self, msg):
        if msg.data >= 0 and msg.data < 1:
            rospy.logerr('Cannot set bandwidth limit below 1Mbps')
            return

        # First, update param server
        rospy.set_param('~tx_bw_limit_mbps', msg.data)

        # Now set from value from param server
        self.set_upload_bw_limit_from_params()

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
            rospy.logerr("Unable to set uploade bandwidth limit: " + str(e))

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

    def __init__(self):
        rospy.init_node(self.NODE_NAME)

        rospy.loginfo("Starting the {} node".format(self.NODE_NAME))

        self.dhcp_enabled = False # initialize to false -- will be updated in set_ips_from_params
        self.tx_byte_cnt_deque = collections.deque(maxlen=2)
        self.rx_byte_cnt_deque = collections.deque(maxlen=2)

        # Initialize from the config file (which should be loaded ahead of this call)
        self.set_ips_from_params()

        self.set_upload_bw_limit_from_params()

        # Public namespace stuff
        rospy.Subscriber('add_ip_addr', String, self.add_ip)
        rospy.Subscriber('remove_ip_addr', String, self.remove_ip)
        rospy.Subscriber('enable_dhcp', Bool, self.enable_dhcp)
        rospy.Subscriber('reset', Reset, self.reset)
        rospy.Subscriber('save_config', Empty, self.save_config)
        rospy.Subscriber('set_tx_bw_limit_mbps', Int32, self.set_upload_bwlimit)

        # Private namespace stuff
        rospy.Subscriber('~reset', Reset, self.reset)
        rospy.Subscriber('~save_config', Empty, self.save_config)

        rospy.Service('ip_addr_query', IPAddrQuery, self.handle_ip_addr_query)
        rospy.Service('bandwidth_usage_query', BandwidthUsageQuery, self.handle_bandwidth_usage_query)

        rospy.Timer(rospy.Duration(self.BANDWIDTH_MONITOR_PERIOD_S), self.monitor_bandwidth_usage)

        self.store_params_publisher = rospy.Publisher('store_params', String, queue_size=1)

        self.run()

if __name__ == "__main__":
    node = NetworkMgr()
