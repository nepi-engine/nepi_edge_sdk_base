import rospy

from std_msgs.msg import Empty, String

'''
Basic interface for the global and private save_config topics.
'''
class SaveCfgIF(object):
    def saveConfig(self, msg):
        if (self.updateParams):
            self.updateParams() # Callback provided by the container class to set values to param server, etc.
        self.store_params_publisher.publish(rospy.get_name())

    def __init__(self, updateParamsCallback=None):
        self.updateParams = updateParamsCallback
        self.store_params_publisher = rospy.Publisher('store_params', String, queue_size=1)
        rospy.Subscriber('save_config', Empty, self.saveConfig)
        rospy.Subscriber('~save_config', Empty, self.saveConfig)
