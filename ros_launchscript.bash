#!/bin/bash


# Check if processes are running - we don't want to start duplicates
IS_RUNNING_ROSCORE=$(ps aux | grep "roscore" | grep -v grep | awk '{print $2}')
IS_RUNNING_LIDAR=$(ps aux | grep "roslaunch urg_node" | grep -v grep | awk '{print $2}')
IS_RUNNING_CAM=$(ps aux | grep "roslaunch usb_cam" | grep -v grep | awk '{print $2}')
IS_RUNNING_SERIAL=$(ps aux | grep "rosrun rosserial_python" | grep -v grep | awk '{print $2}')
IS_RUNNING_GPS=$(ps aux | grep "roslaunch ublox_gps" | grep -v grep | awk '{print $2}')

# Roscore bringup
if [ -z "$IS_RUNNING_ROSCORE" ]
then
    echo "roscore already running, skipping..."
else
    echo "starting roscore"
    screen -d -m roscore
fi
sleep 2


# LIDAR node bringup
if [ -z "$IS_RUNNING_LIDAR" ]
then
    echo "LIDAR node already running, skipping..."
else
    echo "starting LIDAR node"
    screen -d -m roslaunch urg_node urg_lidar.launch
fi
sleep 1


# Camera node bringup
if [ -z "$IS_RUNNING_CAM" ]
then
    echo "USB cam node already running, skipping..."
else
    echo "starting USB cam node"
    screen -d -m roslaunch usb_cam usb_cam-runtime.launch
fi
sleep 1


# GPS node bringup
if [ -z "$IS_RUNNING_GPS" ]
then
    echo "GPS node already running, skipping..."
else
    echo "starting GPS node"
    screen -d -m roslaunch ublox_gps ublox_device.launch param_file_name:=NEO-M8U.yaml
fi
sleep 1


# Serial (Arduino comms) node bringup
if [ -z "$IS_RUNNING_SERIAL" ]
then
    echo "Arduino serial link already running, skipping..."
else
    echo "starting Arduino serial link"
    screen -d -m rosrun rosserial_python serial_node.py _port:=/dev/arduino_nano _baud:=115200
fi
sleep 1
