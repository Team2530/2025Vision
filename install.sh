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

# copy files
cp 2025Vision.service /etc/systemd/system/
chmod 640 /etc/systemd/system/2025Vision.service
systemctl enable 2025Vision

# start if not active
if [[ $(systemctl is-active --quiet 2025Vision) != 0 ]]; then
	systemctl start 2025Vision
fi
