#!/bin/bash
echo '1-2' | sudo tee /sys/bus/usb/drivers/usb/unbind
sleep 1
echo '1-2' | sudo tee /sys/bus/usb/drivers/usb/bind
sleep 2
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2