#!/usr/bin/env python
#TODO License

import rospy
import json
import os
import datetime
import re

from num_sdk_msgs.msg import OotStatus
from num_sdk_base.srv import NepiDataDir, NepiDataDirResponse, ProcessCfgUpdates, ProcessCfgUpdatesResponse
from num_sdk_sb2.srv import LppConfigActionSequence, LppConfigGeofence, LppConfigSmartTriggerRule,\
        LppConfigTaskSchedule, LppConfigSensor, LppConfigSmartTrigger

NEPI_ROOT = "/home/nepi-usr"

class NepiCoreMgr:
    """The NEPI Core Manager Node of the Numurus core SDK.

    The role of this node is to manage all interaction between the
    NEPI-Bot and the rest of the system. This means both relaying
    commands and data from the cloud to the appropriate subsystems as
    well as relaying data from the device to the NEPI-Bot for upload.
    """

    NODE_NAME = "nepi_core_mgr"

    @staticmethod
    def _timestamp():
        return datetime.datetime.now().strftime("%Y_%m_%d_%H%M%S.%f")[:-3]

    @staticmethod
    def _verify_nepi_fs():
        if not (os.path.exists(NEPI_ROOT) and os.path.isdir(NEPI_ROOT)):
            rospy.logfatal("NEPI_ROOT ({}) does not exist or is not a directory!".format(NEPI_ROOT))
            return False

        # TODO: add more once ICD is nailed down.
        subdirs = [
                os.path.join(NEPI_ROOT, "data"),
        ]

        for subdir in subdirs:
            if not (os.path.exists(subdir) and os.path.isdir(subdir)):
                rospy.logfatal("NEPI directory {} does not exist or is not a directory!".format(subdir))
                return False

        return True

    def handle_oot_status(self, msg):
        rospy.loginfo("Received oot_status message, fix time (s): {}".format(msg.navsat_fix_time.secs))

        oot_status_dict = {
                "timestamp": msg.timestamp.secs + msg.timestamp.nsecs/1e9,
                "serial_num": msg.serial_num,
                "sw_rev": msg.sw_rev,
                "navsat_fix_time": msg.navsat_fix_time.secs + msg.navsat_fix_time.nsecs/1e9,
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "heading": msg.heading,
                "batt_charge": msg.batt_charge,
                "bus_voltage": msg.bus_voltage,
                "temperature": msg.temperature,
                "trig_wake_count": msg.trig_wake_count,
                "wake_event_type": msg.wake_event_type,
                "wake_event_id": msg.wake_event_id,
                "task_index": msg.task_index,
                "trig_cfg_index": msg.trig_cfg_index,
                "rule_cfg_index": msg.rule_cfg_index,
                "sensor_cfg_index": msg.sensor_cfg_index,
                "node_cfg_index": msg.node_cfg_index,
                "state_flags": msg.state_flags,
        }

        if not self._verify_nepi_fs():
            rospy.logfatal("Failed to write sys_stat.json due to NEPI FS error (see previous messages).")
            return

        data_dir = os.path.join(NEPI_ROOT, "data", self._timestamp())
        if os.path.exists(data_dir):
            rospy.logwarn("Data directory ({}) already exists, copying files anyway...".format(data_dir))
        else:
            os.mkdir(data_dir)
        self.current_data_dir = data_dir

        with open(os.path.join(data_dir, "sys_stat.json"), "w") as f:
            json.dump(oot_status_dict, f, indent=4)
            f.write("\n")

    def get_current_data_dir(self, req):
        return NepiDataDirResponse(self.current_data_dir)

    def _call_service(self, srv_name, srv_type, srv_args):
        try:
            rospy.wait_for_service(srv_name)
        except rospy.RosException, exc:
            rospy.logwarn("{} failed: {}".format(exc))
            return None
        try:
            proxy = rospy.ServiceProxy(srv_name, srv_type)
            resp = proxy(*srv_args)
        except rospy.ServiceException, exc:
            return None

        return resp

    def _process_one_action_update(self, fn):
        if not re.match("^action_seq_cfg_\\d+\\.json$", fn):
            rospy.logerr("Invalid name for action cfg file: {}".format(fn))
            return

        tmp = re.search("\\d+", fn)
        version_index = int(fn[tmp.start():tmp.end()])

        fullpath = os.path.join(NEPI_ROOT, "action", fn)
        with open(fullpath) as f:
            data = json.load(f)

        action_seq_index = data["action_seq_id"]
        actions = [item["action_id"] for item in data["actions"]]
        max_durations = [item["max_duration"] for item in data["actions"]]

        resp = self._call_service("lpp_config_action_sequence", LppConfigActionSequence,
                (version_index, action_seq_index, actions, max_durations))

        if resp is not None and resp.success:
            rospy.loginfo("Successfully processed {}".format(fn))
        else:
            rospy.logerr("Failed to process {}".format(fn))

    def process_action_updates(self):
        action_updates = os.listdir(os.path.join(NEPI_ROOT, "action"))
        for fn in action_updates:
            self._process_one_action_update(fn)

    def _process_one_geofence_update(self, fn):
        if not re.match("^geofence_cfg_\\d+\\.json$", fn):
            rospy.logerr("Invalid name for geofence cfg file: {}".format(fn))
            return

        tmp = re.search("\\d+", fn)
        version_index = int(fn[tmp.start():tmp.end()])

        fullpath = os.path.join(NEPI_ROOT, "geofence", fn)
        with open(fullpath) as f:
            data = json.load(f)

        polygon_id = data["polygon_id"]
        enabled = data["enabled"]
        lat_vertices = [item["lat"] for item in data["vertices"]]
        lon_vertices = [item["long"] for item in data["vertices"]]

        resp = self._call_service("lpp_config_geofence", LppConfigGeofence,
                (version_index, polygon_id, enabled, lat_vertices, lon_vertices))

        if resp is not None and resp.success:
            rospy.loginfo("Successfully processed {}".format(fn))
        else:
            rospy.logerr("Failed to process {}".format(fn))

    def process_geofence_updates(self):
        geofence_updates = os.listdir(os.path.join(NEPI_ROOT, "geofence"))
        for fn in geofence_updates:
            self._process_one_geofence_update(fn)

    def _process_one_rules_update(self, fn):
        if not re.match("^smarttrig_rule_\\d+\\.json$", fn):
            rospy.logerr("Invalid name for rules cfg file: {}".format(fn))
            return

        tmp = re.search("\\d+", fn)
        version_index = int(fn[tmp.start():tmp.end()])

        fullpath = os.path.join(NEPI_ROOT, "rules", fn)
        with open(fullpath) as f:
            data = json.load(f)

        line = data["rule_id"]
        enabled = data["enabled"]
        smarttrig_ids = [item["smarttrig_id"] for item in data["premises"]]
        # FIXME: deal with this "instance" thing.
        smarttrig_freq_threshes = [item["freq_thresh"] for item in data["premises"]]
        action = data["action_seq_id"]
        min_period = data["min_period"]

        resp = self._call_service("lpp_config_smart_trigger_rule", LppConfigSmartTriggerRule,
                (version_index, line, enabled, smarttrig_ids, smarttrig_freq_threshes, action, min_period))

        if resp is not None and resp.success:
            rospy.loginfo("Successfully processed {}".format(fn))
        else:
            rospy.logerr("Failed to process {}".format(fn))

    def process_rules_updates(self):
        rules_updates = os.listdir(os.path.join(NEPI_ROOT, "rules"))
        for fn in rules_updates:
            self._process_one_rules_update(fn)

    def _process_one_sched_update(self, fn):
        if not re.match("^task_cfg_\\d+\\.json$", fn):
            rospy.logerr("Invalid name for task cfg file: {}".format(fn))
            return

        tmp = re.search("\\d+", fn)
        version_index = int(fn[tmp.start():tmp.end()])

        fullpath = os.path.join(NEPI_ROOT, "sched", fn)
        with open(fullpath) as f:
            data = json.load(f)

        line = data["line"]
        start_time = data["start_time"]
        period = data["period"]
        max_repetition = data["max_repetition"]
        action_seq_index = data["action_seq_id"]

        resp = self._call_service("lpp_config_task_schedule", LppConfigTaskSchedule,
                (version_index, line, start_time, period, max_repetition, action_seq_index))

        if resp is not None and resp.success:
            rospy.loginfo("Successfully processed {}".format(fn))
        else:
            rospy.logerr("Failed to process {}".format(fn))

    def process_sched_updates(self):
        sched_updates = os.listdir(os.path.join(NEPI_ROOT, "sched"))
        for fn in sched_updates:
            self._process_one_sched_update(fn)

    def _process_one_sensors_update(self, fn):
        if not re.match("^sensor_cfg_\\d+\\.json$", fn):
            rospy.logerr("Invalid name for sensor cfg file: {}".format(fn))
            return

        tmp = re.search("\\d+", fn)
        version_index = int(fn[tmp.start():tmp.end()])

        fullpath = os.path.join(NEPI_ROOT, "sensors", fn)
        with open(fullpath) as f:
            data = json.load(f)

        sensor_id = data["sensor_id"]
        config_params = [item["param_id"] for item in data["params"]]
        values = [item["value"] for item in data["params"]]

        resp = self._call_service("lpp_config_sensor", LppConfigSensor,
                (version_index, sensor_id, config_params, values))

        if resp is not None and resp.success:
            rospy.loginfo("Successfully processed {}".format(fn))
        else:
            rospy.logerr("Failed to process {}".format(fn))

    def process_sensors_updates(self):
        sensors_updates = os.listdir(os.path.join(NEPI_ROOT, "sensors"))
        for fn in sensors_updates:
            self._process_one_sensors_update(fn)

    def _process_one_trig_update(self, fn):
        if not re.match("^smarttrig_cfg_\\d+\\.json$", fn):
            rospy.logerr("Invalid name for smarttrig cfg file: {}".format(fn))
            return

        tmp = re.search("\\d+", fn)
        version_index = int(fn[tmp.start():tmp.end()])

        fullpath = os.path.join(NEPI_ROOT, "trig", fn)
        with open(fullpath) as f:
            data = json.load(f)

        smarttrig_id = data["smarttrig_type"]
        smarttrig_type_index = data["instance"]
        config_params = [item["param_id"] for item in data["params"]]
        values = [item["value"] for item in data["params"]]

        resp = self._call_service("lpp_config_smart_trigger", LppConfigSmartTrigger,
                (version_index, smarttrig_id, smarttrig_type_index, config_params, values))

        if resp is not None and resp.success:
            rospy.loginfo("Successfully processed {}".format(fn))
        else:
            rospy.logerr("Failed to process {}".format(fn))

    def process_trig_updates(self):
        trig_updates = os.listdir(os.path.join(NEPI_ROOT, "trig"))
        for fn in trig_updates:
            self._process_one_trig_update(fn)

    def process_cfg_updates(self, req):
        # TODO: for each: archive processed files
        # TODO: for each: file locking
        self.process_action_updates()
        self.process_geofence_updates()
        self.process_rules_updates()
        self.process_sched_updates()
        self.process_sensors_updates()
        self.process_trig_updates()
        return ProcessCfgUpdatesResponse()

    def run(self):
        rospy.spin()

    def __init__(self):
        rospy.init_node(self.NODE_NAME)

        rospy.loginfo("Starting the {} node".format(self.NODE_NAME))

        rospy.Subscriber("oot_status", OotStatus, self.handle_oot_status)

        self.current_data_dir = ""
        rospy.Service("get_current_data_dir", NepiDataDir, self.get_current_data_dir)
        rospy.Service("process_cfg_updates", ProcessCfgUpdates, self.process_cfg_updates)

        self.run()

if __name__ == "__main__":
    node = NepiCoreMgr()
