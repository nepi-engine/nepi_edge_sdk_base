#!/usr/bin/env python
#TODO License

import sys
import numpy as np

import pipeline

from num_sdk_sb2.srv import LppSbandTriggerData

class SBT(pipeline.ServiceDriverNode):
    NODE_TYPE="SBT"
    SRV_NAME="lpp_request_sband_trigger_data"
    SRV_TYPE=LppSbandTriggerData

    @staticmethod
    def parse_raw_data(data):
        ret = np.zeros((len(data), 1, 1, 1))
        for i in range(len(data)):
            ret[i][0][0][0] = data[i]
        return ret

if __name__ == "__main__":
    node = SBT.from_clargs(sys.argv)