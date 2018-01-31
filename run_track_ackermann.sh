#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz

x-terminal-emulator -e "roslaunch mybot_gazebo track_world.launch" #--noclose
sleep 10
x-terminal-emulator -e "roslaunch mybot_control mybot_control.launch" #--noclose
sleep 3
x-terminal-emulator -e "rosrun rqt_gui rqt_gui" #--noclose
sleep 3
x-terminal-emulator -e "rosrun mybot_ackermann ackermann.py"
