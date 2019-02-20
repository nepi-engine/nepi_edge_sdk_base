#!/usr/bin/env python
#TODO License

import sys
import numpy as np

import pipeline

from num_sdk_sb2.srv import LppMicTriggerData

class HMT(pipeline.ServiceDriverNode):
    NODE_TYPE="HMT"
    SRV_NAME="lpp_request_mic_trigger_data"
    SRV_TYPE=LppMicTriggerData

    def parse_raw_data(self, data):
        ret = np.zeros((len(data), 1, 1, 1))
        for i in range(len(data)):
            ret[i][0][0][0] = data[i]
        return ret

if __name__ == "__main__":
    node = HMT.from_clargs(sys.argv)
