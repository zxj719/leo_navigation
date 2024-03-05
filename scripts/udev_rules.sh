#!/bin/bash
sudo chmod 777 /dev/ttyUSB0
echo "remap the device serial port(ttyUSBX) to rplidar"
echo "rplidar usb connection as /dev/rplidar, check it using the command : ls -l /dev | grep ttyUSB"
echo "start copy rplidar.rules to /etc/udev/rules.d/"
source /usr/share/colcon_cd/function/colcon_cd.sh
colcon_cd rplidar_ros
sudo udevadm control --reload && sudo udevadm trigger
echo "finish"
