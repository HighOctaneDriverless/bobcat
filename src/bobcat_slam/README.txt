to get this running start the bobcat_robot_pose_ekf.launch file and after that start the slam_gmapping package

rosrun gmapping slam_gmapping scan:=scan _odom_frame:=odom_combined _base_frame:=base_link _map_frame:=map
