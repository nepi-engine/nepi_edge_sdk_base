import os
import json
import base64
import numbers
import cv2
import numpy as np

import rospy

from num_sdk_base.srv import NepiDataDir
from num_sdk_msgs.srv import NavPosQuery

class PipelineNode(object):
    """Numurus SDK Pipeline Node Abstract Base Class."""

    def __init__(self, typ, inst,
            src_typ=None, src_inst=None,
            msg_in=None, msg_out=None,
            nepi_out=False, change_data=False, node_score=0.0):
        """
        typ: Node type (see ICD).
        inst: Node instance # (see ICD).
        src_typ: Source node type (see ICD).
        src_inst: Source node instance # (see ICD).
        msg_in: Type of ROS msg that the source node produces.
        msg_out: Type of ROS msg that this node should produce.
        nepi_out: Whether or not to write output to the NEPI FS.
        node_score: Used in PIPO scoring.
        """
        self.typ = typ
        self.inst = inst
        self.node_id = "_".join((self.typ, str(self.inst)))

        rospy.init_node(self.node_id)
        rospy.loginfo("Starting the {} node".format(self.node_id))

        self.data_source = self._init_data_source(src_typ, src_inst, msg_in)
        self.data_sink = self._init_data_sink(msg_out)

        self.nepi_out = nepi_out
        self.change_data = change_data
        self.node_score = node_score

    def process(self, msg):
        return msg

    def extract_data(self, msg):
        return np.fromstring(msg.data, np.float32)

    def calculate_quality(self, msg):
        return 0.0

    def _init_data_source(self, src_typ, src_inst, msg_in):
        if None in (src_typ, src_inst, msg_in):
            return None

        return rospy.Subscriber("_".join((src_typ, src_inst)), msg_in, self._handle_input)

    def _init_data_sink(self, msg_out):
        if msg_out is None:
            return None

        # TODO: support configurable queue_size or latch?
        return rospy.Publisher(self.node_id, msg_out, queue_size=1)

    def _handle_input(self, msg):
        msg_out = self.process(msg)
        if self.data_sink is not None:
            self.data_sink.publish(msg_out)
        if self.nepi_out:
            self._nepi_write(msg_out)
    
    def _nepi_write(self, msg):
        data = self.extract_data(msg)
        change_data = self._calculate_diff(data)\
                if (self.change_data and self.last_data is not None)\
                else None
        metadata_file = "{}_meta.json".format(self.node_id)
        data_file = "{}_std.json".format(self.node_id)
        change_data_file = "{}_change.json".format(self.node_id)

        ts = rospy.get_rostime()
        metadata = {
                "node_type": self.typ,
                "instance": self.inst,
                "timestamp": ts.secs + ts.nsecs // 1e6 / 1e3,
                "heading": self._get_current_heading(),
                "quality": self.calculate_quality(data),
                "node_id_score": self.node_score,
                "data_file": data_file
        }
        if change_data is not None:
            metadata["change_data_file"] = change_data_file

        data_dir = self._get_current_data_dir()
        if data_dir == "" or not os.path.isdir(data_dir):
            rospy.logerr("Unable to obtain current data directory, aborting.")
        else:
            with open(os.path.join(data_dir, metadata_file), "w") as f:
                json.dump(metadata, f, indent=4)
            fs_write = cv2.FileStorage(os.path.join(data_dir, data_file), cv2.FILE_STORAGE_WRITE)
            fs_write.write("data", data)
            fs_write.release()
            if change_data is not None:
                with open(os.path.join(data_dir, change_data_file), "w") as f:
                    json.dump(change_data, f, indent=4)

        self.last_data = data

    def _calculate_diff(self, msg):
        # TODO: implement, remember no change case
        raise NotImplementedError()

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

class DriverNode(PipelineNode):
    pass # TODO

#class ProcessingNode(NumurusPipelineNode):
#    """Numurus SDK Pipeline Processing Node.
#
#    This is the base implementation of a processing node for use
#    within the Numurus SDK. It ingests data from a ROS topic, calls
#    a processing routine on this data, then emits the result as a
#    different ROS topic. Subclasses must define
#    """
#
#    def __init__(self, typ, inst, subscriber_tup, publisher_tup):
#        super(ProcessingNode, self).__init__(typ, inst, subscriber_tup, publisher_tup)
#        rospy.spin()
#
#    def _handle_input(self, msg):
#        self.publisher.publish(self.process(msg))
#
#    def process(self, msg):
#        """Turn an input message into an output message.
#
#        This is the meat of the processing node. The single argument
#        is a msg received via subscriber. This routine is responsible
#        for parsing that msg, doing any necessary processing to
#        generate an output msg, and returning that output msg. The
#        default implementation is a simple pass-through.
#        """
#        return msg
#
#class SourceNode(NumurusPipelineNode):
#    """Numurus SDK Pipeline Source Node.
#
#    This is the base implementation of a data source node or driver
#    within the Numurus SDK. It fetches data from a ROS service or
#    other source and emits it as a ROS topic formatted for use within
#    other pipeline nodes.
#    """
#
#    def __init__(self, typ, inst, publisher_tup):
#        super(SourceNode, self).__init__(typ, inst, None, publisher_tup)
#        self.source_init()
#        rospy.spin()
#
#    def source_init(self, *args, **kwargs):
#        """Set up data source.
#
#        Perform whatever additional setup is required to generate
#        data to output. This may include setting a periodic timer,
#        connecting to a signal, or anything else.
#        """
#        raise NotImplementedError("Must be overridden by subclass.")
#
#class SinkNode(NumurusPipelineNode):
#    """Numurus SDK Pipeline Sink Node.
#
#    This is the base implementation of a data sink node within the
#    Numurus SDK. It ingests data from a ROS topic, optionally performs
#    compression and/or differencing and writes the products to the
#    NEPI filesystem interface. Sink nodes are also responsible for
#    assigning each output a quality score for use by the PIPO.
#    """
#
#    def __init__(self, typ, inst, subscriber_tup, diff=False, node_score=0.0):
#        super(SinkNode, self).__init__(typ, inst, subscriber_tup, None)
#        self.diff = diff
#        self.node_score = node_score
#        self.last_data = None
#        rospy.spin()

