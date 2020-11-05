#! /bin/bash

source /opt/numurus/ros/setup.sh

# The sys_env script must exist and be valid. The committed base file is
# (intentionally) not valid because TBD fields are not populated
if [ ! -f /opt/numurus/ros/etc/sys_env.bash ]; then
	echo "ERROR! Could not find /opt/numurus/ros/etc/sys_env.bash"
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

if [ -z "$DEVICE_ID" ]; then
	export DEVICE_ID=$DEVICE_SN
fi

if [ "$SDK_PROJECT" = "TBD" ]; then
	echo "ERROR! SDK_PROJECT must be set in /opt/numurus/ros/sys_env.bash"
	exit 1
fi

if [ "$LAUNCH_FILE" = "TBD" ]; then
	echo "ERROR! LAUNCH_FILE must be set in /opt/numurus/ros/sys_env.bash"
fi

# This is a good place to monitor and purge large log sets
ROS_LOG_SIZE=($(rosclean check))
echo ROS Log Size: $ROS_LOG_SIZE
# A "G" in the size indicates logs are over 1GB, so we purge
if grep -q "G" <<< "$ROS_LOG_SIZE"; then
	echo "Purging ROS logs because they exceed 1GB"
	yes | rosclean purge
fi

roslaunch ${SDK_PROJECT} ${LAUNCH_FILE}
