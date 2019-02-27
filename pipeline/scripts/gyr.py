#!/usr/bin/env python
#TODO License

import sys
import numpy as np

import pipeline

from num_sdk_sb2.srv import LppGyroData

class GYR(pipeline.ServiceDriverNode):
    NODE_TYPE="GYR"
    SRV_NAME="lpp_request_gyro_data"
    SRV_TYPE=LppGyroData

    @staticmethod
    def parse_raw_data(data):
        ret = np.zeros((len(data), 1, 1, 3))
        for i in range(len(data)):
            ret[i][0][0][0] = data[i].x
            ret[i][0][0][1] = data[i].y
            ret[i][0][0][2] = data[i].z
        return ret

if __name__ == "__main__":
    node = GYR.from_clargs(sys.argv)
