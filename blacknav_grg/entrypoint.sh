#!/bin/bash

sudo /lib/systemd/systemd-udevd --daemon
sudo udevadm control --reload-rules 
sudo udevadm trigger
sudo chmod -R 777 /dev

source /opt/ros/jazzy/setup.bash
sudo pigpiod

exec "$@"
