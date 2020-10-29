import rospy

from std_msgs.msg import Empty, String
from num_sdk_msgs.msg import Reset
from num_sdk_msgs.srv import *

'''
Basic interface for the global and private save_config topics.
'''
class SaveCfgIF(object):
    def saveConfig(self, msg):
        if (self.updateParams):
            self.updateParams() # Callback provided by the container class to set values to param server, etc.
        self.store_params_publisher.publish(rospy.get_name()) # Need the fully-qualified namespace name here

    def reset(self, msg):
        reset_proxy = None
        if (msg.reset_type == Reset.USER_RESET):
            reset_proxy = rospy.ServiceProxy('user_reset', FileReset)
        elif (msg.reset_type == Reset.FACTORY_RESET):
            reset_proxy = rospy.ServiceProxy('factory_reset', FileReset)

        ret_val = False
        if (reset_proxy):
            try:
                resp = reset_proxy(rospy.get_name()) # Need the fully-qualified namespace name here
                ret_val = resp.success
            except rospy.ServiceException as e:
                rospy.logerr("%s: service call failed: %s", rospy.get_name(), e)

        if (self.paramsModified):
            self.paramsModified() # Callback provided by container class to update based on param server, etc.

        return ret_val

    def __init__(self, updateParamsCallback=None, paramsModifiedCallback=None):
        self.updateParams = updateParamsCallback
        self.paramsModified = paramsModifiedCallback
        self.store_params_publisher = rospy.Publisher('store_params', String, queue_size=1)
        rospy.Subscriber('save_config', Empty, self.saveConfig)
        rospy.Subscriber('~save_config', Empty, self.saveConfig)
        rospy.Subscriber('reset', Reset, self.reset)
        rospy.Subscriber('~reset', Reset, self.reset)
