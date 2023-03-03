#! /bin/bash

source /opt/nepi/ros/setup.sh

SYS_ENV_FILE=/opt/nepi/sys_env.bash

# The sys_env script must exist and be valid. The committed base file is
# (intentionally) not valid because TBD fields are not populated
if [ ! -f ${SYS_ENV_FILE} ]; then
	echo "ERROR! Could not find ${SYS_ENV_FILE}"
	exit 1
fi

source ${SYS_ENV_FILE}
if [ "$DEVICE_TYPE" = "TBD" ]; then
	echo "ERROR! DEVICE_TYPE must be set in ${SYS_ENV_FILE}"
	exit 1
fi

if [ "$DEVICE_SN" = "TBD" ]; then
	echo "ERROR! DEVICE_SN must be set in ${SYS_ENV_FILE}"
	exit 1
fi

if [ -z "$DEVICE_ID" ]; then
	export DEVICE_ID=$DEVICE_SN
fi

if [ "$ROS1_PACKAGE" = "TBD" ] || [ "$ROS1_LAUNCH_FILE" = "TBD" ]; then
	echo "No ROS1 defs in ${SYS_ENV_FILE}... nothing to launch"
	exit 0
fi

# This is a good place to monitor and purge large log sets
ROS_LOG_SIZE=($(rosclean check))
echo ROS Log Size: $ROS_LOG_SIZE
# A "G" in the size indicates logs are over 1GB, so we purge
if grep -q "G" <<< "$ROS_LOG_SIZE"; then
	echo "Purging ROS logs because they exceed 1GB"
	yes | rosclean purge
fi

roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}
