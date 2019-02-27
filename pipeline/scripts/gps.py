#!/usr/bin/env python
#TODO License

import sys
import numpy as np

import pipeline

from num_sdk_sb2.srv import LppGpsData

class GPS(pipeline.ServiceDriverNode):
    NODE_TYPE="GPS"
    SRV_NAME="lpp_request_gps_data"
    SRV_TYPE=LppGpsData

    @staticmethod
    def parse_raw_data(data):
        ret = np.zeros((len(data), 1, 1, 4))
        for i in range(len(data)):
            ret[i][0][0][0] = data[i].fix_time
            ret[i][0][0][1] = data[i].quality_flag
            ret[i][0][0][2] = data[i].latitude
            ret[i][0][0][3] = data[i].longitude
        return ret

if __name__ == "__main__":
    node = GPS.from_clargs(sys.argv)
