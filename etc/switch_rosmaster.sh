#!/bin/bash

# DEPRECATED - Use the dedicated ROS set_rosmaster topic instead!
# This script does not work for 3DX-CS -- sonar node will fail to launch if you use this script to change rosmaster

# Run this script locally on 3DX or remotely using
# ssh -i ssh_priv_key numurus@<3dx_ip_addr> 'bash -s' < ./switch_rosmaster.sh <new_master_ip>

SYS_ENV_FILE="/opt/nepi/sys_env.bash"
ROS1_LAUNCH_SCRIPT="/opt/nepi/ros/etc/roslaunch.sh"
MASTER_HOST_IP="localhost"

if [ -z $1 ]
then
  echo "Usage:"
  echo " $0 <ros_master>"
  echo " where <ros_master> is \"3DX\" or the IPv4 address of a remote rosmaster"
  exit
elif [ $1 = "3DX" ]
then
  MASTER_HOST_IP="localhost"
else
  ping -c 1 $1
  if [ ! $? -eq 0 ]
  then
    echo "Unable to ping new remote master $1 -- refusing to proceed in case this is a bad IP address"
    exit
  fi
  MASTER_HOST_IP=$1
fi

grep -q "export ROS_MASTER_URI=" ${SYS_ENV_FILE} &&
  sed -i "s/export ROS_MASTER_URI=.*/export ROS_MASTER_URI=http:\/\/${MASTER_HOST_IP}:11311/" ${SYS_ENV_FILE} || echo -e "\nexport ROS_MASTER_URI=http://${MASTER_HOST_IP}:11311" >> ${SYS_ENV_FILE}

if [ $1 = "3DX" ]
then
  sed -i 's/roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE} --wait/roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}/' ${ROS1_LAUNCH_SCRIPT}
else
  sed -i 's/roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}/roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE} --wait/' ${ROS1_LAUNCH_SCRIPT}
fi
