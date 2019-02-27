#!/usr/bin/env python
#TODO License

import sys
import numpy as np

import pipeline

from num_sdk_sb2.srv import LppAccelData, LppGyroData, LppMagData, LppChargeState, LppTemperatureData, LppGpsData

from acl import ACL
from gyr import GYR
from mag import MAG
from bat import BAT
from thr import THR
from gps import GPS

class LPD(pipeline.ServiceDriverCollectionNode):
    NODE_TYPE="LPD"
    SUBNODE_NAMES=("ACL", "GYR", "MAG", "BAT", "THR", "GPS")
    SRV_NAMES=("lpp_request_accel_data", "lpp_request_gyro_data", "lpp_request_mag_data",
            "lpp_request_charge_state_data", "lpp_request_temperature_data", "lpp_request_gps_data")
    SRV_TYPES=(LppAccelData, LppGyroData, LppMagData, LppChargeState, LppTemperatureData, LppGpsData)

    @staticmethod
    def parse_raw_data(name, data):
        if name == "lpp_request_accel_data":
            return ACL.parse_raw_data(data)
        elif name == "lpp_request_gyro_data":
            return GYR.parse_raw_data(data)
        elif name == "lpp_request_mag_data":
            return MAG.parse_raw_data(data)
        elif name == "lpp_request_charge_state_data":
            return BAT.parse_raw_data(data)
        elif name == "lpp_request_temperature_data":
            return THR.parse_raw_data(data)
        elif name == "lpp_request_gps_data":
            return GPS.parse_raw_data(data)
        rospy.logerr("Invalid service name: {}".format(name))
        return None

if __name__ == "__main__":
    node = LPD.from_clargs(sys.argv)
