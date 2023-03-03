if [ -z "$SSH_KEY_PATH" ]; then
	SSH_KEY_PATH=$HOME/.ssh/numurus_3dx_jetson_sshkey
fi

if [ ! -f "$SSH_KEY_PATH" ]; then
	echo "Enter the path to the SSH key for the remote system"
	read -e -p "Enter the path to the SSH key for the remote system
	> " -i $SSH_KEY_PATH SSH_KEY_PATH
	if [ ! -f "$SSH_KEY_PATH" ]; then
		echo "Error: $SSH_KEY_PATH does not exist... exiting"
		exit 1
	fi
fi

if [ -z "$REMOTE_HOST" ]; then
	REMOTE_HOST=192.168.179.102
	read -e -p "Enter the remote hostname or IP address
	> " -i $REMOTE_HOST REMOTE_HOST
fi

# Copy the entire existing folder for archive purposes
NOW=`date +"%F_%H%M%S"`
ARCHIVE_FOLDER=numurus_ros_sdk_archive_${REMOTE_HOST}_$NOW
echo "Archiving the existing installation to $ARCHIVE_FOLDER.tar.gz... this may take a moment"
sleep 3
rsync -avzhe "ssh -i $SSH_KEY_PATH" numurus@192.168.179.102:/opt/nepi/ros $ARCHIVE_FOLDER
tar -czf $ARCHIVE_FOLDER.tar.gz $ARCHIVE_FOLDER
rm -rf $ARCHIVE_FOLDER
