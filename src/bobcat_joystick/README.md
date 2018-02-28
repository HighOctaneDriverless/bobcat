sudo apt-get install ros-kinetic-joy
sudo chmod a+rw /dev/input/js0
rosparam set joy_node/dev "/dev/input/jsX"
rosrun joy joy_node
