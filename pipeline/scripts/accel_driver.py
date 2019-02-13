#!/usr/bin/env python
#TODO License

import sys

import pipeline

from num_sdk_sb2.srv import LppAccelData

class AccelDriver(pipeline.ServiceDriverNode):
    NODE_TYPE="ACL"
    SRV_NAME="lpp_request_accel_data"
    SRV_TYPE=LppAccelData


if __name__ == "__main__":
    node = AccelDriver.from_clargs(sys.argv)
