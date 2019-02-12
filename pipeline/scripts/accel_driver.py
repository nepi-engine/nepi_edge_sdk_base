#!/usr/bin/env python
#TODO License

import rospy
import numpy as np
import argparse

import pipeline

from num_sdk_sb2.srv import LppAccelData
from num_sdk_base.msg import WakeUpEvent, NodeOutput

NODE_TYPE="ACL"

class AccelDriver(pipeline.DriverNode):

    def __init__(self, inst, interval, samples, nepi_out=False, change_data=False):
        super(AccelDriver, self).__init__(NODE_TYPE, inst,
                nepi_out=nepi_out, change_data=change_data, node_score=1.0)

        self.samples = samples
        rospy.Subscriber("wake_up_event", WakeUpEvent, self._get_data)
        rospy.spin()

    def _get_data(self, msg):
        try:
            rospy.wait_for_service("lpp_request_accel_data")
        except rospy.ROSException, exc:
            rospy.logwarn("lpp_request_accel_data failed: {}".format(exc))
            return
        try:
            proxy = rospy.ServiceProxy("lpp_request_accel_data", LppAccelData)
            resp = proxy(self.samples)
        except rospy.ServiceException, exc:
            rospy.logwarn("lpp_request_accel_data failed: {}".format(exc))
            return

        msg = self.data_to_msg(resp.data)

        self._handle_input(msg)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
            "--inst",
            type=int,
            required=True,
            dest="inst",
            help="Node instance #"
            )
    parser.add_argument(
            "--interval",
            type=float,
            default=1.0,
            dest="interval",
            help="Data fetch interval (s)"
            )
    parser.add_argument(
            "--samples",
            type=int,
            default=10,
            dest="samples",
            help="Number of samples to fetch"
            )
    parser.add_argument(
            "--nepi-out",
            action="store_true",
            dest="nepi_out",
            help="Output data to NEPI FS"
            )
    # roslaunch passes these whether we want them or not
    parser.add_argument("__name")
    parser.add_argument("__log")

    args = parser.parse_args()

    node = AccelDriver(args.inst, args.interval, args.samples, args.nepi_out)
