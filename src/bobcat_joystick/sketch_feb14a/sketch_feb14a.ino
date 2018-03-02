/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */


/*
 * To get this running you need to start a roscore and run 
 * rosrun rosserial_python serial_node.py /dev/ttyACM0
 * replace ttyACM0 with the name of your device 
*/
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>

ros::NodeHandle  nh;

Servo steering;
Servo motor;
int input_timeout;
int max_input_timeout = 100;

void steering_cb( const std_msgs::Int8& cmd_msg){
  int out = - cmd_msg.data;
  out = out /2;
  if(out > 90){
    out = 90;
  }else if(out < -90){
    out = -90;
  }
  out = 90 + out;
  steering.write(out); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void esc_cb( const std_msgs::Int8& cmd_msg){
  int output = 0;
  input_timeout = 0;
  output = - cmd_msg.data;

  if(output >23){
    output = 23;
  }
  else if(output < -20){
    output = -20;
  }
    output = 90 + output;

  motor.write(output); //set servo angle, should be from 0-180  
}


ros::Subscriber<std_msgs::Int8> sub("steering", steering_cb);
ros::Subscriber<std_msgs::Int8> sub2("motor", esc_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  
  steering.attach(9); //attach it to pin 9
  motor.attach(10);
  input_timeout = 0;
  max_input_timeout = 150;
}

void loop(){
  nh.spinOnce();
  input_timeout ++;
  if(input_timeout >= max_input_timeout){
    motor.write(90);
  }
  
  delay(1);
}














