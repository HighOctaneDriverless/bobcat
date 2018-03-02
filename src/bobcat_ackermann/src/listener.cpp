#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
using namespace std;
#include <iostream>



std_msgs::Float64 v;
std_msgs::Float64 v1;
std_msgs::Float64 e;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
ROS_INFO("I heard: [%s]", msg->data.c_str());
const std_msgs::String::ConstPtr& msg
*/
void getVel(const sensor_msgs::JointState &velocity)
{
    v.data = velocity.velocity[1];
    v1.data = velocity.velocity[4];
    /*if(v.data == v1.data){
    ROS_INFO("Velocity left rear: [%f], Velocity right rear: [%f], true", v.data, v1.data);}
    else {ROS_INFO("Velocity left rear: [%f], Velocity right rear: [%f], false", v.data, v1.data);}*/
}

void setVel (const std_msgs::Float64 &effort)
{
    e.data = effort.data;
    ROS_INFO("Effort: [%f]",e.data);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

    //Geschwindigkeit der joint states holen
  ros::Subscriber sub = n.subscribe("/bobcat/joint_states", 10, getVel);
  //Publishen der aktuellen GEwschwindigkeit an state
  ros::Publisher pub = n.advertise<std_msgs::Float64>("/state",10);
  ros::Subscriber sub1 = n.subscribe("/control_effort",10,setVel);
  ros::Publisher lefteffort = n.advertise<std_msgs::Float64>("/bobcat/left_rear_wheel_effort_controller/command",10);
  ros::Publisher righteffort = n.advertise<std_msgs::Float64>("/bobcat/right_rear_wheel_effort_controller/command",10);
  ros::Publisher setpoint = n.advertise<std_msgs::Float64>("/setpoint",10);
    ros::Rate loop_rate(10);

  while(ros::ok()){

        std_msgs::Float64 bub;
        bub.data = 2.0;
        setpoint.publish(bub);
        pub.publish(v);
        lefteffort.publish(e);
        righteffort.publish(e);
        loop_rate.sleep();
        ros::spin();
  }

  return 0;
}
