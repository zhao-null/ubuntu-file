#!/bin/bash

## change these to whatever you actually need
command1="sudo -S chmod 777 /dev/ttyUSB*<<zcy; bash"
command2="sleep 1s;roslaunch rplidar_ros view_rplidar.launch; bash"
command3="sleep 3s;roslaunch simu_follower laser_follower.launch; bash"
## command4="sleep 1s;rosrun serial serial_example /ttyUSB0 115200; bash"

## Modify terminator's config
sed -i.bak "s#COMMAND1#$command1#; s#COMMAND2#$command2#; s#COMMAND3#$command3#; s#COMMAND4#$command4#;" ~/.config/terminator/config

## Launch a terminator instance using the new layout
terminator -l default

## Return the original config file
mv ~/.config/terminator/config.bak ~/.config/terminator/config

