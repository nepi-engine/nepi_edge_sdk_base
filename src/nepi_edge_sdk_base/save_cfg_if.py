#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import rospy

from std_msgs.msg import Empty, String
from nepi_ros_interfaces.msg import Reset
from nepi_ros_interfaces.srv import *

'''
Basic interface for the global and private save_config topics.
'''
class SaveCfgIF(object):
    def saveConfig(self, msg):
        if (self.updateParams):
            self.updateParams() # Callback provided by the container class to set values to param server, etc.
        if self.namespace != None:
            self.store_params_publisher.publish(self.namespace)
        else:
            self.store_params_publisher.publish(rospy.get_name()) # Need the fully-qualified namespace name here

    def resetConfig(self, msg):
        reset_proxy = None
        if (msg.reset_type == Reset.USER_RESET):
            reset_proxy = rospy.ServiceProxy('user_reset', FileReset)
        elif (msg.reset_type == Reset.FACTORY_RESET):
            reset_proxy = rospy.ServiceProxy('factory_reset', FileReset)

        ret_val = False
        if (reset_proxy):
            try:
            
                if self.namespace != None:
                    resp = reset_proxy(self.namespace)
                else:
                    resp = reset_proxy(rospy.get_name()) # Need the fully-qualified namespace name here
                ret_val = resp.success
            except rospy.ServiceException as e:
                rospy.logerr("%s: service call failed: %s", rospy.get_name(), e)

        if (self.paramsModified):
            self.paramsModified() # Callback provided by container class to update based on param server, etc.

        return ret_val

    def __init__(self, updateParamsCallback=None, paramsModifiedCallback=None, namespace = None):
        self.updateParams = updateParamsCallback
        self.paramsModified = paramsModifiedCallback
        self.store_params_publisher = rospy.Publisher('store_params', String, queue_size=1)
        self.namespace = namespace
        
        if namespace != None:
            save_topic = namespace + '/save_config'
            reset_topic = namespace + '/reset_config'
        else:
            save_topic = '~save_config'
            reset_topic = '~reset_config'   

        
        rospy.Subscriber(save_topic, Empty, self.saveConfig)
        rospy.Subscriber(reset_topic, Reset, self.resetConfig)
        
        rospy.Subscriber('save_config', Empty, self.saveConfig)
        rospy.Subscriber('reset_config', Reset, self.resetConfig)
