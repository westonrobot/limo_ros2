#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules

echo  'KERNEL=="ttyUSB*", KERNELS=="3-1.4*"，ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777",SYMLINK+="ttylimo"' >/etc/udev/rules.d/limo-usb.rules


service udev reload
sleep 2
service udev restart

