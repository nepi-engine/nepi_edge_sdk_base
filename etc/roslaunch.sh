#! /bin/bash

source /opt/numurus/ros/setup.bash

# The sys_env script must exist and be valid. The committed base file is 
# (intentionally) not valid because TBD fields are not populated
if [ ! -f /opt/numurus/ros/etc/sys_env.bash ]; then
	echo "ERROR! Could not find /opt/numurus/ros/sys_env.bash"
	exit 1
fi

source /opt/numurus/ros/etc/sys_env.bash
if [ "$DEVICE_TYPE" = "TBD" ]; then
	echo "ERROR! DEVICE_TYPE must be set in /opt/numurus/ros/sys_env.bash"
	exit 1
fi

if [ "$DEVICE_SN" = "TBD" ]; then
	echo "ERROR! DEVICE_SN must be set in /opt/numurus/ros/sys_env.bash"
	exit 1
fi

if [ "$SDK_PROJECT" = "TBD" ]; then
	echo "ERROR! SDK_PROJECT must be set in /opt/numurus/ros/sys_env.bash"
	exit 1
fi

roslaunch ${SDK_PROJECT} ${DEVICE_TYPE}.launch