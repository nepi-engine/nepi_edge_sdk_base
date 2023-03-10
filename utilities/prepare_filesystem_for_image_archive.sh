#!/bin/bash

# This script cleans up a working system ahead of a release archive (e.g., via l4t flash.sh)
# It is intended to run locally on a Numurus system (3DX, S2X, etc)
# It must be run under sudo or as root
DATA_CONTENTS=/mnt/nepi_storage/data/*
NUMURUS_ROS_LOG_CONTENTS=/home/numurus/.ros/log/*
ROOT_ROS_LOG_CONTENTS=/root/.ros/log/*

NUMURUS_ROS_FOLDER=/opt/nepi/ros

NEPI_LOG_CONTENTS=/opt/nepi/nepi/nepi-bot/log/*
NEPI_DB_CONTENTS=/opt/nepi/nepi/nepi-bot/db/nepibot.db
NEPI_HB_CONTENTS=/opt/nepi/nepi/nepi-bot/hb/*
NEPI_LB_CONTENTS=/opt/nepi/nepi/nepi-bot/lb/*

if [ "$EUID" -ne 0 ]
then
  echo "Must run this script as root (or under sudo)"
  exit
fi

echo "Shutting down ROS to close current logs"
systemctl stop roslaunch

echo "Cleaning out the data folder"
rm -rf $DATA_CONTENTS
echo "... done"

echo "Cleaning out the ROS logs"
rm -rf $NUMURUS_ROS_LOG_CONTENTS $ROOT_ROS_LOG_CONTENTS
echo "... done"

echo "Checking for user-custom config settings and reverting interactively"
USER_CFG_ARRAY=($(find $NUMURUS_ROS_FOLDER | grep -F .user))

for i in "${USER_CFG_ARRAY[@]}"
do
  CFG_FILE=$(basename "$i" .user)
  CFG_PATH=$(dirname "$i")
  echo "Should we revert $CFG_FILE to factory settings? (y or n)"
  read should_revert
  if [ $should_revert == y ]
  then
    echo "Ok... reverting $CFG_FILE"
    ln -sf ${CFG_PATH}/${CFG_FILE}.num_factory ${CFG_PATH}/${CFG_FILE}
    rm ${CFG_PATH}/${CFG_FILE}.user
  else
    echo "Ok... skipping $USER_FILE"
  fi
done
echo "... done"

echo "Clearing out NEPI temporary files"
rm -rf $NEPI_LOG_CONTENTS $NEPI_DB_CONTENTS $NEPI_HB_CONTENTS $NEPI_LB_CONTENTS
echo "... done"

# TODO: Clear out the ~ (home folder) of source code etc. Need to identify what
# is required in that home folder first to make image detection labels work properly
# and ideally fix the catkin installation so this is no longer needed.

echo "Filesystem prep complete... can proceed to archive this filesystem for release"
