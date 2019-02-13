import os
import json
import base64
import numbers
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import argparse

import rospy

from num_sdk_base.srv import NepiDataDir
from num_sdk_base.msg import WakeUpEvent, NodeOutput
from num_sdk_msgs.srv import NavPosQuery

class PipelineNode(object):
    """Numurus SDK Pipeline Node Abstract Base Class."""

    def __init__(self, typ, inst,
            src_typ=None, src_inst=None,
            nepi_out=False, node_score=0.0):
        """
        typ: Node type (see ICD).
        inst: Node instance # (see ICD).
        src_typ: Source node type (see ICD).
        src_inst: Source node instance # (see ICD).
        nepi_out: Whether or not to write output to the NEPI FS.
        node_score: Used in PIPO scoring.
        """
        self.typ = typ
        self.inst = inst
        self.node_id = "_".join((self.typ, str(self.inst)))

        rospy.init_node(self.node_id)
        rospy.loginfo("Starting the {} node".format(self.node_id))

        self.data_source = self._init_data_source(src_typ, src_inst)
        self.data_sink = self._init_data_sink()

        self.nepi_out = nepi_out
        self.node_score = node_score

        self.previous_data = None

        self.bridge = CvBridge()

    def process(self, data):
        """Process input data and return output data.

        This method may be overridden by subclasses to perform some
        processing on input data objects. The "data" argument is
        assumed to be a 3d numpy array (equivalent to an OpenCV
        matrix), in which the outermost dimension is time and the two
        inner dimensions are unspecified at this level.
        """
        return data

    def derive_change_data(self, current_data, previous_data):
        """Derive change data product.

        This method may be overridden by subclasses to enable the use
        of change data products, which may in turn reduce bandwidth
        requirements for data transmission.

        The two arguments may be assumed to be the two most recent
        data products, though note that "previous_data" will be None
        in the case of the first data product.

        This method should return either:
        * None: Indicates no data change product should be used.
        * "NC": Indicates that current_data and previous_data are
                identical.
        * A change data representation suitable for passing to Python's
          FileIO.write() method.
        """
        return None

    def data_to_msg(self, data):
        return NodeOutput(
                [self.bridge.cv2_to_imgmsg(np.array(data[i], ndmin=2, dtype=np.float32)) for i in range(len(data))]
        )

    def msg_to_data(self, msg):
        return np.array(
                [self.bridge.imgmsg_to_cv2(msg.time_series[i]) for i in range(len(msg.time_series))]
        )

    def calculate_quality(self, msg):
        return 0.0

    def _init_data_source(self, src_typ, src_inst):
        if None in (src_typ, src_inst):
            return None

        return rospy.Subscriber("_".join((src_typ, src_inst)), NodeOutput, self._handle_input)

    def _init_data_sink(self):
        # TODO: support configurable queue_size or latch?
        return rospy.Publisher(self.node_id, NodeOutput, queue_size=1)

    def _handle_input(self, msg):
        data = self.msg_to_data(msg)
        data_out = self.process(data)
        msg_out = self.data_to_msg(data_out)
        if self.data_sink is not None:
            self.data_sink.publish(msg_out)
        if self.nepi_out:
            self._nepi_write(msg_out)
    
    def _nepi_write(self, msg):
        data = self.msg_to_data(msg)
        change_data = self.derive_change_data(data, self.previous_data)

        metadata_file = "{}_meta.json".format(self.node_id)
        data_file = "{}_std.json".format(self.node_id)
        change_data_file = "{}_change.json".format(self.node_id)
        if change_data is None:
            change_data_file = "null"
        elif change_data == "NC":
            change_data_file = "NC"

        ts = rospy.get_rostime()
        metadata = {
                "node_type": self.typ,
                "instance": self.inst,
                "timestamp": ts.secs + ts.nsecs // 1e6 / 1e3,
                "heading": self._get_current_heading(),
                "quality": self.calculate_quality(data),
                "node_id_score": self.node_score,
                "data_file": data_file,
                "change_file": change_data_file
        }

        data_dir = self._get_current_data_dir()
        if data_dir == "" or not os.path.isdir(data_dir):
            rospy.logerr("Unable to obtain current data directory, aborting.")
        else:
            with open(os.path.join(data_dir, metadata_file), "w") as f:
                json.dump(metadata, f, indent=4)
                f.write("\n")
            fs_write = cv2.FileStorage(os.path.join(data_dir, data_file+"?base64"), cv2.FILE_STORAGE_WRITE)
            fs_write.write("data", data)
            fs_write.release()
            if change_data is not None and change_data != "NC":
                # FIXME: Untested.
                with open(os.path.join(data_dir, change_data_file), "w") as f:
                    f.write(change_data)

        self.previous_data = data

    def _get_current_data_dir(self):
        try:
            rospy.wait_for_service("get_current_data_dir")
        except rospy.ROSException, exc:
            rospy.logerr("get_current_data_dir failed: {}".format(exc))
            return ""

        try:
            proxy = rospy.ServiceProxy("get_current_data_dir", NepiDataDir)
            resp = proxy()
        except rospy.ServiceException, exc:
            rospy.logerr("get_current_data_dir failed: {}".format(exc))
            return ""

        return resp.dir

    def _get_current_heading(self):
        try:
            rospy.wait_for_service("nav_pos_query")
        except rospy.ROSException, exc:
            rospy.logerr("nav_pos_query failed: {}".format(exc))
            return -1

        try:
            proxy = rospy.ServiceProxy("nav_pos_query", NavPosQuery)
            resp = proxy(rospy.get_rostime())
        except rospy.ServiceException, exc:
            rospy.logwarn("nav_pos_query failed: {}".format(exc))
            return -1

        return resp.nav_pos.heading

class ServiceDriverNode(PipelineNode):
    """Driver that gets data from a ROS Service."""

    # These must be defined by subclass.
    NODE_TYPE=None
    SRV_NAME=None
    SRV_TYPE=None

    def __init__(self, inst, interval, samples,
            nepi_out=False, node_score=0.0):
        super(ServiceDriverNode, self).__init__(self.NODE_TYPE, inst,
                nepi_out=nepi_out, node_score=node_score)

        self.samples = samples
        rospy.Subscriber("wake_up_event", WakeUpEvent, self._get_data)
        rospy.spin()

    @classmethod
    def from_clargs(cls, argv):
        # Remove executable name arg.
        argv = argv[1:]
        # Comb out roslaunch args.
        argv = [arg for arg in argv if not arg.startswith(("__name", "__log"))]

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
        parser.add_argument(
                "--node-score",
                type=float,
                default=0.0,
                dest="node_score",
                help="Node score"
                )

        args = parser.parse_args(argv)

        return cls(args.inst, args.interval, args.samples, nepi_out=args.nepi_out, node_score=args.node_score)

    def _get_data(self, msg):
        try:
            rospy.wait_for_service(self.SRV_NAME)
        except rospy.ROSException, exc:
            rospy.logwarn("{} failed: {}".format(self.SRV_NAME, exc))
            return
        try:
            proxy = rospy.ServiceProxy(self.SRV_NAME, self.SRV_TYPE)
            resp = proxy(self.samples)
        except rospy.ServiceException, exc:
            rospy.logwarn("{} failed: {}".format(self.SRV_NAME, exc))
            return

        msg = self.data_to_msg(resp.data)
        self._handle_input(msg)

