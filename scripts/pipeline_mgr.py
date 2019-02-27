#!/usr/bin/env python
#TODO License

import os
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
        for k in self.processes:
            self.processes[k].stop()

    def launch_node(self, launcher, nodename, node):
        try:
            return launcher.launch(node)
        except:
            rospy.logerr("Failed to launch node {}".format(nodename))
            return None

    def __init__(self):
        rospy.init_node(self.NODE_NAME)

        rospy.loginfo("Starting the {} node".format(self.NODE_NAME))

        # FIXME: needs to be read from env variables
        namespace = "/{}/{}/{}".format(os.environ["ROOTNAME"],
                                       os.environ["DEVICE_TYPE"],
                                       os.environ["DEVICE_SN"])

        # TODO: launch pipeline nodes!
#       acl1 = roslaunch.core.Node("num_sdk_base",
#               "acl.py",
#               namespace=namespace,
#               args="--inst=1 --nepi-out",
#               respawn=True, respawn_delay=10.0)
#       gyr1 = roslaunch.core.Node("num_sdk_base",
#               "gyr.py",
#               namespace=namespace,
#               args="--inst=1 --nepi-out",
#               respawn=True, respawn_delay=10.0)
#       mag1 = roslaunch.core.Node("num_sdk_base",
#               "mag.py",
#               namespace=namespace,
#               args="--inst=1 --nepi-out",
#               respawn=True, respawn_delay=10.0)
#       bat1 = roslaunch.core.Node("num_sdk_base",
#               "bat.py",
#               namespace=namespace,
#               args="--inst=1 --nepi-out",
#               respawn=True, respawn_delay=10.0)
#       thr1 = roslaunch.core.Node("num_sdk_base",
#               "thr.py",
#               namespace=namespace,
#               args="--inst=1 --nepi-out",
#               respawn=True, respawn_delay=10.0)
#       gps1 = roslaunch.core.Node("num_sdk_base",
#               "gps.py",
#               namespace=namespace,
#               args="--inst=1 --nepi-out",
#               respawn=True, respawn_delay=10.0)
        lpd1 = roslaunch.core.Node("num_sdk_base",
                "lpd.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        self.nodes = {
                "lpd1": lpd1
        }

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        self.processes = {k: self.launch_node(launch, k, self.nodes[k]) for k in self.nodes}

        # TODO: define config interface

        self.run()
        self.cleanup()

if __name__ == "__main__":
    node = PipelineMgr()
