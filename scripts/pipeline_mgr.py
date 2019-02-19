#!/usr/bin/env python
#TODO License

import rospy
import roslaunch

class PipelineMgr:
    """The Pipeline Manager Node of the Numurus core SDK.

    The role of this node is to launch all pipeline nodes and provide
    a corresponding configuration interface.
    """

    NODE_NAME = "pipeline_mgr"

    def run(self):
        rospy.spin()

    def cleanup(self):
        self.process.stop()

    def __init__(self):
        rospy.init_node(self.NODE_NAME)

        rospy.loginfo("Starting the {} node".format(self.NODE_NAME))

        # FIXME: needs to be read from env variables
        namespace = "/numurus/sb1/0000_01"

        # TODO: launch pipeline nodes!
        accel_driver = roslaunch.core.Node("num_sdk_base",
                "acl.py",
                namespace=namespace,
                args="--inst=1 --interval=10.0 --nepi-out",
                respawn=True, respawn_delay=10.0)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        self.process = launch.launch(accel_driver)

        # TODO: define config interface

        self.run()
        self.cleanup()

if __name__ == "__main__":
    node = PipelineMgr()
