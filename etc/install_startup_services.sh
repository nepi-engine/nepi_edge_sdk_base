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

# This script is to be run after building nepi_base_ws to install and enable 
# the systemd service files associated with NEPI.

# Must be run as root (or via 'sudo')
SYSTEMD_SERVICE_PATH=/etc/systemd/system

cp /opt/nepi/ros/etc/roslaunch.service $SYSTEMD_SERVICE_PATH
systemctl enable roslaunch
#systemctl status roslaunch # Some status printout

cp /opt/nepi/ros/etc/num_gpsd.service $SYSTEMD_SERVICE_PATH
systemctl enable num_gpsd
#systemctl status num_gpsd # Some status printout

echo "Script complete... you must still edit /opt/nepi/sys_env.bash in order to launch NEPI"
