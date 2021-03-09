#! /bin/bash

# This file is in the num_sdk_base project because nepi-edge-sdk is intended to become
# open-source and general purpose, and the following is reliant on other Numurus infrastructure
# that isn't necessarily available on a generic target.

# Must source the ROS setup first, as it overwrites important messages
source /opt/numurus/ros/setup.sh
source /opt/numurus/nepi/nepi-edge-sdk/setup.bash

# The sys_env script must exist and be valid. The committed base file is
# (intentionally) not valid because TBD fields are not populated
if [ ! -f /opt/numurus/ros/etc/sys_env.bash ]; then
	echo "ERROR! Could not find /opt/numurus/ros/etc/sys_env.bash"
	exit 1
fi

source /opt/numurus/ros/etc/sys_env.bash
if [ "$ROOTNAME" = "TBD" ]; then
	echo "ERROR! ROOTNAME must be set in /opt/numurus/ros/sys_env.bash"
	exit 1
fi

if [ "$DEVICE_TYPE" = "TBD" ]; then
	echo "ERROR! DEVICE_TYPE must be set in /opt/numurus/ros/sys_env.bash"
	exit 1
fi

if [ "$DEVICE_SN" = "TBD" ]; then
	echo "ERROR! DEVICE_SN must be set in /opt/numurus/ros/sys_env.bash"
	exit 1
fi

if [ -z "$DEVICE_ID" ]; then
	export DEVICE_ID=$DEVICE_SN
fi

# Set the required
export NEPI_EDGE_ROS_BRIDGE_NS=/numurus/$DEVICE_TYPE/$DEVICE_ID
export NEPI_EDGE_ROS_BRIDGE_PARAM_FILE=/opt/numurus/ros/etc/nepi_edge_ros_bridge/nepi_edge_ros_bridge.yaml

roslaunch nepi_edge_ros_bridge nepi_edge_ros_bridge.launch
