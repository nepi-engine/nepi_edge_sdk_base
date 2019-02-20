# This is the base sys_env.bash file that sets up the ROS namespace
# of the device. Variables marked "to_be_populated" must be set when
# this is installed on a target. The launch scripts are designed to reject
# unset values 

# The ROOTNAME is used as the first namespace element. It is numurus by default, but can be overridden here
export ROOTNAME=numurus

# The DEVICE_TYPE must match an installed <DEVICE_TYPE>.launch file. Examples include DEVICE_TYPE=3dsc
export DEVICE_TYPE=TBD

# The DEVICE_SN must be set and should be a unique serial number/identifier for each build.
export DEVICE_SN=TBD

# The SDK_PROJECT must be set as the project that contains <DEVICE_TYPE>.launch
export SDK_PROJECT=TBD