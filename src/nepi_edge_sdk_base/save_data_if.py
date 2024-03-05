#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import datetime

import rospy

from std_msgs.msg import String
from nepi_ros_interfaces.msg import SaveData, SaveDataRate, SaveDataStatus
from nepi_ros_interfaces.srv import DataProductQuery, DataProductQueryResponse, SystemStorageFolderQuery, SystemStorageFolderQueryResponse

'''
Basic interface for the global and private save_data topics.
'''
class SaveDataIF(object):
    def save_data_callback(self, msg):
        # Policy is to save calibration whenever data saving is enabled
        if (self.save_continuous is False) and (msg.save_continuous is True):
            self.needs_save_calibration = True

        self.save_continuous = msg.save_continuous
        self.save_raw = msg.save_raw

        rospy.set_param('~save_data_continuous', self.save_continuous)
        rospy.set_param('~save_data_raw', self.save_raw)

        self.publish_save_status()

    def save_data_prefix_pub_ns_callback(self, msg):
        self.save_data_prefix = msg.data

        # Mark for calibration save if this will be in a subdirectory
        # TODO: Better would be if we detected that this was a *new* subdirectory, but that logic is more complicated and
        # saving calibration more often than necessary seems pretty benign
        if '/' in self.save_data_prefix:
            self.needs_save_calibration = True
        # TODO: Should we monitor here to ensure that a new folder gets created by system_mgr if required according to the new prefix before proceeding?
        
        self.publish_save_status()

    def save_data_prefix_priv_ns_callback(self, msg):
        self.save_data_prefix = msg.data

        if self.save_data_root_directory is None:
            return # No data directory

        # Now ensure the directory exists if this prefix defines a subdirectory
        full_path = os.path.join(self.save_data_root_directory, self.save_data_prefix)
        parent_path = os.path.dirname(full_path)
        #rospy.logwarn("DEBUG!!!! Computed full path " + full_path + " and parent path " + parent_path)
        if not os.path.exists(parent_path):
            rospy.loginfo("Creating new data subdirectory " + parent_path)
            os.makedirs(parent_path)
            os.chown(parent_path, self.DATA_UID, self.DATA_GID)
            # Mark that we should save calibration to the new folder
            self.needs_save_calibration = True

        self.publish_save_status()

    def save_data_rate_callback(self, msg):
        if (msg.data_product == msg.ALL_DATA_PRODUCTS):
            for d in self.data_rate_dict:
                # Respect the max save rate
                self.data_rate_dict[d][0] = msg.save_rate_hz if msg.save_rate_hz <= self.data_rate_dict[d][2] else self.data_rate_dict[d][2]

        elif (msg.data_product in self.data_rate_dict):
            self.data_rate_dict[msg.data_product][0] = msg.save_rate_hz if msg.save_rate_hz <= self.data_rate_dict[msg.data_product][2] else self.data_rate_dict[msg.data_product][2]
        else:
            rospy.logerr("%s is not a known data product", msg.data_product)

    def query_data_products_callback(self, req):
        return_list = []
        for d in self.data_rate_dict:
            return_list.append(SaveDataRate(data_product = d, save_rate_hz = self.data_rate_dict[d][0]))

        return DataProductQueryResponse(return_list)

    def publish_save_status(self):
        current_save_data = SaveData(save_continuous = self.save_continuous, save_raw = self.save_raw)
        if self.save_data_root_directory is None:
            self.save_data_status_pub.publish(current_data_dir = "", save_data = current_save_data)
        else:
            current_dir = os.path.dirname(os.path.join(self.save_data_root_directory, self.save_data_prefix))
            self.save_data_status_pub.publish(current_data_dir = current_dir, save_data = current_save_data)

    def data_product_should_save(self, data_product_name):
        # If saving is disabled for this node, then it is not time to save this data product!
        if not self.save_continuous:
            return False

        if data_product_name not in self.data_rate_dict:
            rospy.logwarn("Unknown data product %s", data_product_name)
            return False

        save_rate = self.data_rate_dict[data_product_name][0]
        if save_rate == 0.0:
            return False

        save_period = 1.0 / save_rate
        now = rospy.get_rostime().to_sec()
        elapsed = now - self.data_rate_dict[data_product_name][1]
        if (elapsed >= save_period):
            self.data_rate_dict[data_product_name][1] = now
            return True

        return False
    
    def data_product_saving_enabled(self, data_product_name):
        # If saving is disabled for this node, then no data products are saving
        if not self.save_continuous:
            return False

        if data_product_name not in self.data_rate_dict:
            rospy.logwarn("Unknown data product %s", data_product_name)
            return False

        save_rate = self.data_rate_dict[data_product_name][0]
        return (save_rate > 0.0)

    def calibration_should_save(self):
        needs_cal = False
        if self.needs_save_calibration is True:
            self.needs_save_calibration = False
            needs_cal = True
        return needs_cal

    def get_timestamp_string(self):
        return datetime.datetime.now().strftime('%Y-%m-%dT%H%M%S.%f')[:-3]

    def get_filename_path_and_prefix(self):
        if self.save_data_root_directory is None:
            return ""
        return os.path.join(self.save_data_root_directory, self.save_data_prefix)

    def get_full_path_filename(self, timestamp_string, identifier, extension):
        if self.save_data_root_directory is None:
            return ""
        return os.path.join(self.save_data_root_directory, self.save_data_prefix + timestamp_string + "_" + identifier + "." + extension)

    def __init__(self, data_product_names=None):
        # First thing, need to get the data folder
        try:
            rospy.wait_for_service('system_storage_folder_query', 10.0)
            system_storage_folder_query = rospy.ServiceProxy('system_storage_folder_query', SystemStorageFolderQuery)
            self.save_data_root_directory = system_storage_folder_query('data').folder_path
        except Exception as e:
            rospy.logwarn("Failed to obtain system data folder: " + str(e))
            self.save_data_root_directory = None # Flag it as non-existent
            return

        # Ensure the data folder exists with proper ownership
        if not os.path.exists(self.save_data_root_directory):
            rospy.logwarn("Reported data folder does not exist... data saving is disabled")
            self.save_data_root_directory = None # Flag it as non-existent
            return # Don't enable any of the ROS interface stuff

        # And figure out user/group so that we know what ownership to create subfolders with
        stat_info = os.stat(self.save_data_root_directory)
        self.DATA_UID = stat_info.st_uid
        self.DATA_GID = stat_info.st_gid

        self.data_rate_dict = {}
        for d in data_product_names:
            self.data_rate_dict[d] = [1.0, 0.0, 100.0] # Default to 1Hz save rate, max rate = 100.0Hz

        self.save_continuous = rospy.get_param('~save_data_continuous', False)
        self.save_raw = rospy.get_param('~save_data_raw', False)
        # And force them onto the server in case they weren't already there
        rospy.set_param('~save_data_continuous', self.save_continuous)
        rospy.set_param('~save_data_raw', self.save_raw)

        self.save_data_prefix = ''
        
        self.needs_save_calibration = self.save_continuous

        # Setup subscribers -- public and private versions
        rospy.Subscriber('save_data', SaveData, self.save_data_callback)
        rospy.Subscriber('save_data_prefix', String, self.save_data_prefix_pub_ns_callback)
        rospy.Subscriber('save_data_rate', SaveDataRate, self.save_data_rate_callback)

        rospy.Subscriber('~save_data', SaveData, self.save_data_callback)
        rospy.Subscriber('~save_data_prefix', String, self.save_data_prefix_priv_ns_callback)
        rospy.Subscriber('~save_data_rate', SaveDataRate, self.save_data_rate_callback)

        rospy.Service('~query_data_products', DataProductQuery, self.query_data_products_callback)

        self.save_data_status_pub = rospy.Publisher('~save_data_status', SaveDataStatus, queue_size = 5)
