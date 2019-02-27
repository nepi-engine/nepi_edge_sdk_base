#!/usr/bin/env python
#TODO License

import sys
import numpy as np

import pipeline

from num_sdk_sb2.srv import LppMagData

class MAG(pipeline.ServiceDriverNode):
    NODE_TYPE="MAG"
    SRV_NAME="lpp_request_mag_data"
    SRV_TYPE=LppMagData

    @staticmethod
    def parse_raw_data(data):
        ret = np.zeros((len(data), 1, 1, 3))
        for i in range(len(data)):
            ret[i][0][0][0] = data[i].x
            ret[i][0][0][1] = data[i].y
            ret[i][0][0][2] = data[i].z
        return ret

if __name__ == "__main__":
    node = MAG.from_clargs(sys.argv)
