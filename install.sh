#!/bin/bash

# require system priveledges
if [[ $UID != 0 ]]; then
	echo "This script requires system priveledges."
	echo "Please authenticate to continue."
	sudo -K
	exec sudo "$0" "$@"
	exit 1
else
	echo "Priveledges acquired."
fi

# symlink script
echo "Installing service"

if [[ -f "/usr/local/bin/2025Vision.sh" ]] then
	rm /usr/local/bin/2025Vision.sh
fi

ln -s $PWD/2025Vision.sh /usr/local/bin
chmod +x /usr/local/bin/2025Vision.sh

# copy service
cp 2025Vision.service /etc/systemd/system/
chmod 640 /etc/systemd/system/2025Vision.service
systemctl enable 2025Vision

# start if not active
if [[ $(systemctl is-active 2025Vision) != "active" ]]; then
	echo "Starting service"
	systemctl start 2025Vision
else # restart
	echo "Restarting service"
	systemctl restart 2025Vision
fi
