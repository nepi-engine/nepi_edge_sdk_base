echo 'This script will delete all files from the remote 3DX data directory'
echo 'Enter c to first download all data files to the current directory and then delete'
echo 'Enter d to delete without download'
echo 'or any other key to exit:'

read option

if [ $option == "c" ] 
then
  echo "Downloading all files locally and then deleting"
  if [ ! -d ./3dx_download ] 
  then
    mkdir ./3dx_download
    if [ ! -d ./3dx_download ] 
    then
      echo "Error: Unable to create local 3dx_download directory for data dump... exiting"
      exit 1
    fi
  fi
  scp -i numurus_3dx_jetson_sshkey numurus@192.168.179.102:/home/numurus_user/data/* ./3dx_download
  ssh -i numurus_3dx_jetson_sshkey numurus@192.168.179.102 'rm -rf /home/numurus_user/data/*'
elif [ $option == "d" ]
then
  echo "Deleting files... "
  ssh -i numurus_3dx_jetson_sshkey numurus@192.168.179.102 'rm -rf /home/numurus_user/data/*'
else
  echo "Exiting script by request... goodbye"
  exit 0
fi

