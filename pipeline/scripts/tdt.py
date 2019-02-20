#!/usr/bin/env python
#TODO License

import sys
import numpy as np

import pipeline

from num_sdk_sb2.srv import LppTxdrTriggerData

class TDT(pipeline.ServiceDriverNode):
    NODE_TYPE="TDT"
    SRV_NAME="lpp_request_txdr_trigger_data"
    SRV_TYPE=LppTxdrTriggerData

    def parse_raw_data(self, data):
        ret = np.zeros((len(data), 1, 1, 1))
        for i in range(len(data)):
            ret[i][0][0][0] = data[i]
        return ret

if __name__ == "__main__":
    node = TDT.from_clargs(sys.argv)
