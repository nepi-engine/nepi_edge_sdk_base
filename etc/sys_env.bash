# This is the base sys_env.bash file that sets up the ROS namespace
# of the device. Variables marked "to_be_populated" must be set when
# this is installed on a target. The launch scripts are designed to reject
# unset values

# The ROOTNAME is used as the first namespace element. It is numurus by default, but can be overridden here
export ROOTNAME=numurus

# The DEVICE_TYPE represents the "name" of the device
export DEVICE_TYPE=TBD

# The DEVICE_SN must be set and should be a unique serial number/identifier for each system.
export DEVICE_SN=TBD

# The DEVICE_ID is included in the device ROS namespace. If left unset, it will be assigned to the S/N
export DEVICE_ID=device_0

# The SDK_PROJECT must be set as the project that contains <DEVICE_TYPE>.launch
export SDK_PROJECT=TBD

# The launch file must be installed in the namespace of the SDK_PROJECT
export LAUNCH_FILE=TBD
