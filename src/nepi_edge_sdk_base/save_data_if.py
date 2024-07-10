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
import time

import rospy

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.msg import SaveData, SaveDataRate, SaveDataStatus
from nepi_ros_interfaces.srv import DataProductQuery, DataProductQueryResponse, SystemStorageFolderQuery, SystemStorageFolderQueryResponse

'''
Basic interface for the global and private save_data topics.
'''
class SaveDataIF(object):
    save_continuous = False
    save_raw = False
    save_path = None
    save_data_prefix = ""
    save_data_subfolder = ""    
    snapshot_dict = dict()
    
    def save_data_callback(self, msg):
        # Policy is to save calibration whenever data saving is enabled
        if (self.save_continuous is False) and (msg.save_continuous is True):
            self.needs_save_calibration = True

        save_continuous = msg.save_continuous
        save_raw = msg.save_raw
        self.save_data_enable(save_continuous, save_raw)
        
    def save_data_callback(self, msg):
        if (self.save_continuous is False) and (msg.save_continuous is True):
            self.needs_save_calibration = True

        save_continuous = msg.save_continuous
        save_raw = msg.save_raw
        self.save_data_enable(save_continuous, save_raw)
        

    def save_data_enable(self, save_continuous = False, save_raw = False):
        self.save_continuous = save_continuous
        self.save_raw = save_raw
        rospy.set_param('~save_data_continuous', save_continuous)
        self.save_continuous = save_continuous
        rospy.set_param('~save_data_raw', save_raw)
        self.save_raw = save_raw
        self.publish_save_status()

    def snapshot_callback(self,msg):
        rospy.loginfo("Recieved Snapshot Trigger")
        data_rate_dict = rospy.get_param('~data_rate_dict',self.init_data_rate_dict)
        for data_product_name in data_rate_dict.keys():
            save_rate = data_rate_dict[data_product_name][0]
            enabled = (save_rate > 0.0)
            if enabled:
                triggered = self.snapshot_dict[data_product_name] = True


    def save_data_prefix_pub_ns_callback(self, msg):
        new_prefix = msg.data
        self.update_save_data_path_and_prefix(new_prefix)

    def save_data_prefix_priv_ns_callback(self, msg):
        if self.save_data_root_directory is None:
            return # No data directory
        new_prefix = msg.data
        self.update_save_data_path_and_prefix(new_prefix)

    def update_save_data_path_and_prefix(self,save_data_prefix = ""):
        if '\\' not in save_data_prefix:
            if save_data_prefix.find('/') == -1:
                subfolder = ""
                prefix = save_data_prefix
            else:
                prefix_split = save_data_prefix.rsplit('/',1)
                subfolder = prefix_split[0]
                prefix = prefix_split[1]
            # Now ensure the directory exists if this prefix defines a subdirectory
        else:
            subfolder = ""
            prefix = ""
        if subfolder != "" and self.save_data_root_directory != None:
            full_path = os.path.join(self.save_data_root_directory, subfolder)
        elif self.save_data_root_directory != None:
            full_path = self.save_data_root_directory
        else:
            full_path = ""
        #rospy.logwarn("DEBUG!!!! Computed full path " + full_path + " and parent path " + parent_path)
        if not os.path.exists(full_path):
            rospy.loginfo("Creating new data subdirectory " + full_path)
            try:
                os.makedirs(full_path)
                os.chown(full_path, self.DATA_UID, self.DATA_GID)
                # Mark that we should save calibration to the new folder
                self.save_path = full_path
                self.save_data_subfolder  = subfolder
            except Exception as e:
                self.save_path = self.save_data_root_directory # revert to root folder
                self.save_data_subfolder  = ""
                rospy.loginfo("Could not create save folder " + subfolder + str(e) )
        else:
            self.save_path = full_path
            self.save_data_subfolder  = subfolder
        self.save_data_prefix = prefix

        self.needs_save_calibration = True
        self.publish_save_status()

    def save_data_rate_callback(self, msg):
        data_product = msg.data_product
        save_rate_hz = msg.save_rate_hz
        self.update_save_data_rate(data_product,save_rate_hz)

    def update_save_data_rate(self,data_product,save_rate_hz=0):
        save_all = SaveDataRate().ALL_DATA_PRODUCTS
        data_rate_dict = rospy.get_param('~data_rate_dict',self.init_data_rate_dict)
        if (data_product == save_all):
            for d in data_rate_dict:
                # Respect the max save rate
                data_rate_dict[d][0] = save_rate_hz if save_rate_hz <= data_rate_dict[d][2] else data_rate_dict[d][2]
        elif (data_product in data_rate_dict):
            data_rate_dict[data_product][0] = save_rate_hz if save_rate_hz <= data_rate_dict[data_product][2] else data_rate_dict[data_product][2]
        else:
            rospy.logerr("%s is not a known data product", data_product)           
        rospy.set_param('~data_rate_dict',data_rate_dict)
        self.publish_save_status()
        


    def query_data_products_callback(self, req):
        return_list = []
        data_rate_dict = rospy.get_param('~data_rate_dict',self.init_data_rate_dict)
        for d in data_rate_dict:
            return_list.append(SaveDataRate(data_product = d, save_rate_hz = data_rate_dict[d][0]))

        return DataProductQueryResponse(return_list)

    def publish_save_status(self):
        data_rate_dict = rospy.get_param('~data_rate_dict',self.init_data_rate_dict)
        data_rate_dict_str = "["
        ind = 0
        for data_name in data_rate_dict:
            if ind != 0:
                sep = ","
            else:
                sep = ""
            data_rate_str = str(data_rate_dict[data_name][0])
            data_rate_dict_str = data_rate_dict_str + sep + "[" + data_name + "," + data_rate_str + "]"
            ind = ind + 1
        data_rate_dict_str = data_rate_dict_str + "]"
        current_save_data = SaveData(save_continuous = self.save_continuous, save_raw = self.save_raw)
        if self.save_data_root_directory is None:
            self.save_data_status_pub.publish(current_data_dir = "", current_folder_prefix = self.save_data_subfolder, current_filename_prefix = self.save_data_prefix, save_data_rates = data_rate_dict_str, save_data = current_save_data)
        else:
            self.save_data_status_pub.publish(current_data_dir = self.save_data_root_directory, current_folder_prefix = self.save_data_subfolder, current_filename_prefix = self.save_data_prefix, save_data_rates = data_rate_dict_str, save_data = current_save_data)

    def data_product_should_save(self, data_product_name):
        # If saving is disabled for this node, then it is not time to save this data product!
        data_rate_dict = rospy.get_param('~data_rate_dict',self.init_data_rate_dict)
        if not self.save_continuous:
            return False

        if data_product_name not in data_rate_dict:
            rospy.logwarn("Unknown data product %s", data_product_name)
            return False

        save_rate = data_rate_dict[data_product_name][0]
        if save_rate == 0.0:
            return False

        save_period = 1.0 / save_rate
        now = rospy.get_rostime().to_sec()
        elapsed = now - data_rate_dict[data_product_name][1]
        if (elapsed >= save_period):
            data_rate_dict[data_product_name][1] = now
            rospy.set_param('~data_rate_dict',data_rate_dict)
            return True
        return False


    def data_product_snapshot_enabled(self, data_product_name):
        enabled = self.snapshot_dict[data_product_name]
        return enabled

    def data_product_snapshot_reset(self, data_product_name):
        self.snapshot_dict[data_product_name] = False


    def data_product_snapshot_reset(self, data_product_name):
        triggered = self.snapshot_dict[data_product_name]
        self.snapshot_dict[data_product_name] = False
        return triggered
    

    def save_data_reset_callback(self,reset_msg):
        rospy.loginfo("Recieved save data reset msg")
        rospy.set_param('~data_rate_dict',self.init_data_rate_dict)
        self.publish_save_status()

    def save_data_reset_factory_callback(self,reset_msg):
        rospy.loginfo("Recieved save data factory reset msg")
        rospy.set_param('~data_rate_dict',self.factory_data_rate_dict)
        self.publish_save_status()


    def registerDataProduct(self, data_product_name):
        data_rate_dict = rospy.get_param('~data_rate_dict',self.init_data_rate_dict)
        if data_product_name not in data_rate_dict.keys():
            data_rate_dict[data_product_name] = [1.0, 0.0, 100.0] # Default to 1Hz save rate, max rate = 100.0Hz
            rospy.set_param('~data_rate_dict',data_rate_dict)
        self.publish_save_status()
    
    def data_product_saving_enabled(self, data_product_name):
        # If saving is disabled for this node, then no data products are saving
        data_rate_dict = rospy.get_param('~data_rate_dict',self.init_data_rate_dict)
        if not self.save_continuous:
            return False

        if data_product_name not in data_rate_dict:
            rospy.logwarn("Unknown data product %s", data_product_name)
            return False

        save_rate = data_rate_dict[data_product_name][0]
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
        if self.save_path is None:
            return ""
        return os.path.join(self.save_path, self.save_data_prefix)

    def get_full_path_filename(self, timestamp_string, identifier, extension):
        if self.save_path is None:
            return ""
        spacer = ""
        if self.save_data_prefix != "":
            if self.save_data_prefix[-1] != "_":
                spacer = "_"
        filename = os.path.join(self.save_path, self.save_data_prefix + spacer + timestamp_string + "_" + identifier + "." + extension)
        #rospy.loginfo("******* save data filename: " + filename)
        return filename


    def __init__(self, data_product_names=None):
        if data_product_names != None:
            data_products_str = str(data_product_names)
        else:
            data_products_str = "None"
        rospy.loginfo("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
        rospy.loginfo("Starting Save_Data_IF with data products: " + data_products_str)
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

        self.factory_data_rate_dict= {}
        for d in data_product_names:
            self.factory_data_rate_dict[d] = [0.0, 0.0, 100.0] # Default to 1Hz save rate, set last save = 0.0, max rate = 100.0Hz
            self.snapshot_dict[d] = False
        
        self.init_data_rate_dict = rospy.get_param('~data_rate_dict',self.factory_data_rate_dict)
        rospy.set_param('~data_rate_dict',self.init_data_rate_dict)
        


        self.save_continuous = rospy.get_param('~save_data_continuous', False)
        self.save_raw = rospy.get_param('~save_data_raw', False)
        # And force them onto the server in case they weren't already there
        rospy.set_param('~save_data_continuous', self.save_continuous)
        rospy.set_param('~save_data_raw', self.save_raw)

        self.save_data_prefix = ""
        self.save_data_subfolder = ""
        self.save_path = self.save_data_root_directory
        
        self.needs_save_calibration = self.save_continuous

        # Setup subscribers -- global and local versions
        rospy.Subscriber('save_data', SaveData, self.save_data_callback)
        rospy.Subscriber('snapshot_trigger', Empty, self.snapshot_callback)
        rospy.Subscriber('save_data_prefix', String, self.save_data_prefix_pub_ns_callback)
        rospy.Subscriber('save_data_rate', SaveDataRate, self.save_data_rate_callback)

        rospy.Subscriber('~save_data', SaveData, self.save_data_callback)
        rospy.Subscriber('~snapshot_trigger', Empty, self.snapshot_callback)
        rospy.Subscriber('~save_data_prefix', String, self.save_data_prefix_pub_ns_callback)
        rospy.Subscriber('~save_data_rate', SaveDataRate, self.save_data_rate_callback)
        rospy.Subscriber('~save_data_reset', Empty, self.save_data_reset_callback)
        rospy.Subscriber('~save_data_reset_factory', Empty, self.save_data_reset_factory_callback)

        rospy.Service('~query_data_products', DataProductQuery, self.query_data_products_callback)

        self.save_data_status_pub = rospy.Publisher('~save_data_status', SaveDataStatus, queue_size=1, latch=True)
        time.sleep(1)
        self.publish_save_status()
