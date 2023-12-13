##
## NEPI Dual-Use License
## Project: nepi_edge_sdk_base
##
## This license applies to any user of NEPI Engine software
##
## Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
## see https://github.com/numurus-nepi/nepi_edge_sdk_base
##
## This software is dual-licensed under the terms of either a NEPI software developer license
## or a NEPI software commercial license.
##
## The terms of both the NEPI software developer and commercial licenses
## can be found at: www.numurus.com/licensing-nepi-engine
##
## Redistributions in source code must retain this top-level comment block.
## Plagiarizing this software to sidestep the license obligations is illegal.
##
## Contact Information:
## ====================
## - https://www.numurus.com/licensing-nepi-engine
## - mailto:nepi@numurus.com
##
##
OPENCV_PKG=opencv # Desktop
#OPENCV_PKG=opencv4 # Jetson

g++ --std=c++11 -Ofast -DUNDERWATER_IMG_ENHANCER_TEST_APP `pkg-config --cflags $OPENCV_PKG` -I/usr/include/opencv4/ -I../../include ./underwater_img_enhancer.cpp `pkg-config --libs $OPENCV_PKG` -o underwater_img_enhancer_test_app
