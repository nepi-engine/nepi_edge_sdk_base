#!/usr/bin/env python
#TODO License

import sys
import numpy as np

import pipeline

from num_sdk_sb2.srv import LppChargeState

class BAT(pipeline.ServiceDriverNode):
    NODE_TYPE="BAT"
    SRV_NAME="lpp_request_charge_state_data"
    SRV_TYPE=LppChargeState

    @staticmethod
    def parse_raw_data(data):
        ret = np.zeros((len(data), 1, 1, 1))
        for i in range(len(data)):
            ret[i][0][0][0] = data[i]
        return ret

if __name__ == "__main__":
    node = BAT.from_clargs(sys.argv)
