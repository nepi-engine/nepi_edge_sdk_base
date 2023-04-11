import os
import datetime

import rospy

from std_msgs.msg import String
from nepi_ros_interfaces.msg import SaveData, SaveDataRate, SaveDataStatus
from nepi_ros_interfaces.srv import DataProductQuery, DataProductQueryResponse

'''
Basic interface for the global and private save_data topics.
'''
class SaveDataIF(object):
    SAVE_DATA_ROOT_DIRECTORY = '/mnt/nepi_storage/data' # TODO: This should probably be configurable and/or queried from system_mgr.
    DATA_UID = 1000 # nepi
    DATA_GID = 130 # sambashare # TODO This is very fragile

    def save_data_callback(self, msg):
        # Policy is to save calibration whenever data saving is enabled
        if (self.save_continuous is False) and (msg.save_continuous is True):
            self.needs_save_calibration = True

        self.save_continuous = msg.save_continuous
        self.save_raw = msg.save_raw

        rospy.set_param('~save_data_continuous', self.save_continuous)
        rospy.set_param('~save_data_raw', self.save_raw)

        self.publish_save_status()

    def save_data_prefix_callback(self, msg):
        self.save_data_prefix = msg.data

        # Now ensure the directory exists if this prefix defines a subdirectory
        full_path = self.SAVE_DATA_ROOT_DIRECTORY + '/' + self.save_data_prefix

        parent_path = os.path.dirname(full_path)
        if not os.path.exists(os.path.dirname(parent_path)):
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
        self.save_data_status_pub.publish(current_data_dir = self.save_data_dir, save_data = current_save_data)

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
        return self.SAVE_DATA_ROOT_DIRECTORY + '/' + self.save_data_prefix

    def get_full_path_filename(self, timestamp_string, identifier, extension):
        return self.SAVE_DATA_ROOT_DIRECTORY + '/' + self.save_data_prefix + timestamp_string + "_" + identifier + "." + extension

    def __init__(self, data_product_names=None):
        self.data_rate_dict = {}
        for d in data_product_names:
            self.data_rate_dict[d] = [1.0, 0.0, 100.0] # Default to 1Hz save rate, max rate = 100.0Hz

        self.save_continuous = rospy.get_param('~save_data_continuous', False)
        self.save_raw = rospy.get_param('~save_data_raw', False)
        # And force them onto the server in case they weren't already there
        rospy.set_param('~save_data_continuous', self.save_continuous)
        rospy.set_param('~save_data_raw', self.save_raw)

        self.save_data_prefix = ''
        self.save_data_dir = self.SAVE_DATA_ROOT_DIRECTORY

        self.needs_save_calibration = self.save_continuous

        # Setup subscribers -- public and private versions
        rospy.Subscriber('save_data', SaveData, self.save_data_callback)
    	rospy.Subscriber('save_data_prefix', String, self.save_data_prefix_callback)
    	rospy.Subscriber('save_data_rate', SaveDataRate, self.save_data_rate_callback)

        rospy.Subscriber('~save_data', SaveData, self.save_data_callback)
    	rospy.Subscriber('~save_data_prefix', String, self.save_data_prefix_callback)
    	rospy.Subscriber('~save_data_rate', SaveDataRate, self.save_data_rate_callback)

        rospy.Service('~query_data_products', DataProductQuery, self.query_data_products_callback)

        self.save_data_status_pub = rospy.Publisher('~save_data_status', SaveDataStatus, queue_size = 5)

        # Ensure the data folder exists with proper ownership
        if not os.path.exists(self.SAVE_DATA_ROOT_DIRECTORY):
            os.makedirs(self.SAVE_DATA_ROOT_DIRECTORY)
            os.chown(self.SAVE_DATA_ROOT_DIRECTORY, self.DATA_UID, self.DATA_GID)
