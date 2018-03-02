sudo apt-get install ros-kinetic-joy
#ls /dev/input/
#sudo jstest /dev/input/jsX
sudo chmod a+rw /dev/input/js0
rosparam set joy_node/dev "/dev/input/jsX"
rosrun joy joy_node

#if xbox is not in /dev/input
sudo apt-add-repository -y ppa:rael-gc/ubuntu-xboxdrv
sudo apt-get update
sudo apt-get install ubuntu-xboxdrv
