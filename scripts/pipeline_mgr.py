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
        namespace = "/numurus/sb1/0000_01"

        # TODO: launch pipeline nodes!
        acl1 = roslaunch.core.Node("num_sdk_base",
                "acl.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        gyr1 = roslaunch.core.Node("num_sdk_base",
                "gyr.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        mag1 = roslaunch.core.Node("num_sdk_base",
                "mag.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        bat1 = roslaunch.core.Node("num_sdk_base",
                "bat.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        thr1 = roslaunch.core.Node("num_sdk_base",
                "thr.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        gps1 = roslaunch.core.Node("num_sdk_base",
                "gps.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        hmt1 = roslaunch.core.Node("num_sdk_base",
                "hmt.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        tdt1 = roslaunch.core.Node("num_sdk_base",
                "tdt.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        sbt1 = roslaunch.core.Node("num_sdk_base",
                "sbt.py",
                namespace=namespace,
                args="--inst=1 --nepi-out",
                respawn=True, respawn_delay=10.0)
        self.nodes = {
                "acl1": acl1,
                "gyr1": gyr1,
                "mag1": mag1,
                "bat1": bat1,
                "thr1": thr1,
                "gps1": gps1,
                "hmt1": hmt1,
                "tdt1": tdt1,
                "sbt1": sbt1,
        }

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        self.processes = {k: self.launch_node(launch, k, self.nodes[k]) for k in self.nodes}

        # TODO: define config interface

        self.run()
        self.cleanup()

if __name__ == "__main__":
    node = PipelineMgr()
