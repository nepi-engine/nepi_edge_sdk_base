#!/usr/bin/env python
#TODO License

import sys
import numpy as np

import pipeline

from num_sdk_sb2.srv import LppTemperatureData

class THR(pipeline.ServiceDriverNode):
    NODE_TYPE="THR"
    SRV_NAME="lpp_request_temperature_data"
    SRV_TYPE=LppTemperatureData

    @staticmethod
    def parse_raw_data(data):
        ret = np.zeros((len(data), 1, 1, 1))
        for i in range(len(data)):
            ret[i][0][0][0] = data[i]
        return ret

if __name__ == "__main__":
    node = THR.from_clargs(sys.argv)
