Getting Started!

2018_01_26_2330
Benjamin

------------------------------------

INSTALLATION:

1)
Install ROS Kinetic Kame (Tutorials on ros.org: <http://www.ros.org/install/>)

Create catkin Workspace (catkin_ws) (Tutorials on ros.org: <http://wiki.ros.org/ROS/Tutorials>)
/catkin_ws/src
(this is the folder all packeges go into)


2) Edit .bashrc (BE CAREFUL, mistakes here may crash your Linux)
$ gedit .bashrc

add the following two lines at the very end of your .bashrc file (otherwise you need to run these two lines every time you open a new Terminal window to have access to ROS commands):
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
--> after that you need to "Log Out..." and log in again (so that the edited .bashrc file is loaded)


3) 
Install GIT
$ sudo apt-get install git


4)
Install ROS_Controller_Manager
$ sudo apt-get install ros-kinetic-controller-manager

5)
Install ROS_Control
$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers

6)
Install Gazebo_Control 
$ sudo apt-get install ros-kinetic-gazebo-ros-control


------------------------------------

WORKFLOW:

A) Load new Project

$ cd ~/catkin_ws/src/
alternatively:
right-click on src-folder -> open Terminal 

$ git clone <LINK from github.com of our project> (e.g. bobcat: https://github.com/HighOctaneDriverless/bobcat.git)

or if you are not yet in src-folder:
$ git clone <LINK> <dir> 
	link: [see above]
	dir: ~/catkin_ws/src/

Every time something in src-folder is changed:
~/catkin_ws $ catkin_make


Because the recent branch is not Master (but "branch"):
~/catkin_ws/src/bobcat $ git checkout branch
(CHANGE BRANCH)

~/catkin_ws $ catkin_make



B) Start simulator:
$ roslaunch bobcat_gazebo bobcat_world.launch

C) Start Controller:
$ roslaunch bobcat_control bobcat_control.launch

D) Publish messages on certain Topic (this is the "easy" way using the RQT tool; alternatively in Terminal)
	- $ rosrun rqt_gui rqt_gui
	- blanc window should open
	- in Menu-Bar: "Plug-Ins" --> "Topics" --> "Message Publisher"
	- in Message Publisher:
		- Topic: */command
			(choose e.g. /bobcat/left_steering_hinge_position_controller/command)
		- Type: [don't change]
		- Frequ.: 100 Hz
			--> add (green plus)
		- Activate checkbox, open dropdown, edit expression (e.g. 0.2)



--------------------------------------------------------------------------

Additional general information


If Gazebo broke and you need to restart it you might need to terminate all running processes:
$ killall gzclient
$ killall gzserver
$ kill roscore
