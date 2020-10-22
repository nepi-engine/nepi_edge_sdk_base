import os
import datetime

import rospy

from std_msgs.msg import String
from num_sdk_msgs.msg import SaveData, SaveDataRate, SaveDataStatus
from num_sdk_msgs.srv import DataProductQuery, DataProductQueryResponse

'''
Basic interface for the global and private save_data topics.
'''
class SaveDataIF(object):
    SAVE_DATA_ROOT_DIRECTORY = '/home/numurus_user/data'

    def save_data_callback(self, msg):
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

        self.publish_save_status()

    def save_data_rate_callback(self, msg):
        if (msg.data_product == msg.ALL_DATA_PRODUCTS):
            for d in self.data_rate_dict:
                self.data_rate_dict[d][0] = msg.save_rate_hz

        elif (msg.data_product in self.data_rate_dict):
            self.data_rate_dict[msg.data_product][0] = msg.save_rate_hz
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

    def get_timestamp_string(self, tstamp_ros):
        tstamp_py = datetime.datetime.fromtimestamp(tstamp_ros.to_sec())
        return tstamp_py.strftime('%Y-%m-%dT%H%M%S.%f')[:-3]

    def get_filename_path_and_prefix(self):
        return self.SAVE_DATA_ROOT_DIRECTORY + '/' + self.save_data_prefix

    def __init__(self, data_product_names=None):
        self.data_rate_dict = {}
        for d in data_product_names:
            self.data_rate_dict[d] = [1.0, 0.0] # Default to 1Hz

        self.save_continuous = rospy.get_param('~save_data_continuous', False)
        self.save_raw = rospy.get_param('~save_data_raw', False)
        # And force them onto the server in case they weren't already there
        rospy.set_param('~save_data_continuous', self.save_continuous)
        rospy.set_param('~save_data_raw', self.save_raw)

        self.save_data_rate = 1.0
        self.save_data_prefix = ''
        self.save_data_dir = self.SAVE_DATA_ROOT_DIRECTORY

        # Setup subscribers -- public and private versions
        rospy.Subscriber('save_data', SaveData, self.save_data_callback)
    	rospy.Subscriber('save_data_prefix', String, self.save_data_prefix_callback)
    	rospy.Subscriber('save_data_rate', SaveDataRate, self.save_data_rate_callback)

        rospy.Subscriber('~save_data', SaveData, self.save_data_callback)
    	rospy.Subscriber('~save_data_prefix', String, self.save_data_prefix_callback)
    	rospy.Subscriber('~save_data_rate', SaveDataRate, self.save_data_rate_callback)

        rospy.Service('~query_data_products', DataProductQuery, self.query_data_products_callback)

        self.save_data_status_pub = rospy.Publisher('save_data_status', SaveDataStatus, queue_size = 5)
