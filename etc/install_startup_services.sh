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
