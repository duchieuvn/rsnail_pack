#!/bin/bash


ros2 run ros2_qtcanbus qtcanbus_sender --ros-args -p canbus_plugin:=peakcan -p canbus_interface:=usb1 -p can_type:=motor --ros-args --log-level debug &
ros2 run ros2_qtcanbus qtcanbus_sender --ros-args -p canbus_plugin:=peakcan -p canbus_interface:=usb0 -p can_type:=sensor --ros-args --log-level debug &

