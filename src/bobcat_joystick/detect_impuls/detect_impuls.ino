/*
 * To get this running you need to start a roscore and run 
 * rosrun rosserial_python serial_node.py /dev/ttyUSB1
 * replace ttyUSB1 with the name of your device 
*/

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#if defined(USE_USBCON)
  // Arduino Leonardo USB Serial Port
  #define SERIAL_CLASS Serial_
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;

Servo steering;
Servo motor;
//this is used to set speed output to 0 if there is no new input for more than max_input_timeout loops
//input_timout set to value higher than max for initialization
int input_timeout = 6;
int max_input_timeout = 5;
//set max_pwm values to prevent car from going "berserker"
const int max_forward_pwm = 17;
const int max_backward_pwm = 17;


const int  wheel_encoder_ticks = 61;  //ticks counted for one wheel turn
const float wheel_circumference = 0.335; //in m
const float dist_per_tick = wheel_circumference / wheel_encoder_ticks; //travelled distance per tick

const byte interruptPin1 = 2;
const byte interruptPin2 = 3;

int count_left = 0;
int count_right = 0;
float distance_traveled = 0;
float x_d = 0;

//array to track distance traveled the last array_size loops 
const int array_size = 20;  // size of 
float distance_buffer [array_size] = {0};

//values for pid controller
const float windeup_threshold = 0.02;
const float integral_max_threshold = 20;
float speed_integral = 0;

//values from ros
float desired_speed = 0;

std_msgs::Float64 speed_msg;
ros::Publisher pub_speed("/bobcat/speed", &speed_msg);
std_msgs::Float64 dist_msg;
ros::Publisher pub_dist("/bobcat/distance", &dist_msg);

void setup()
{
  // ros stuff
  ros::Subscriber<std_msgs::Float64> sub("/bobcat/ackermann_steer/command", steering_cb);
  ros::Subscriber<std_msgs::Float64> sub2("/bobcat/ackermann_speed/command", speed_cb);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.advertise(pub_speed);
  nh.advertise(pub_dist);
    
  nh.loginfo("Arduino speed_control setup");
  

  //servo and motor controller
  steering.attach(9); //attach it to pin 9
  motor.attach(10); //attach it to pin 10

  //intterrupts to detect the wheel encoder ticks
  attachInterrupt(digitalPinToInterrupt(interruptPin1), count_detected_left, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), count_detected_right, FALLING);

}

//callback for interrupt left odom 
void count_detected_left(){
  count_left += 1;  
}
//callback for interrupt right odom 
void count_detected_right(){
  count_right += 1;  
}

//computes the absolute driven distance based on the odometer ticks and the distance traveled with one tick
float compute_distance(){
  int ticks_traveled = count_left + count_right;
  //devide it by 2 as we have two odom sensors
  distance_traveled = ticks_traveled * dist_per_tick / 2.0;
  return distance_traveled;
}

//shifts the array one element to the right and updates for index 0 the newest value
void add_value_to_array(float distance_new){
  for(int i = array_size -1; i >=  1; i--){
    distance_buffer[i] = distance_buffer[i - 1];
  }
  distance_buffer[0] = distance_new;
}

//this only works if i get the loop to run at exactly 20hz or 50ms for every step
float compute_speed(){
  //compute median over 5 time_steps (1second)
  int median = 5;
  float x_d = distance_buffer[0] - distance_buffer[median -1];
  x_d = x_d / (median *0.05); 
  return x_d;   
}


//pid controller that takes the desired speed beeing published by ros and computes the output value going to the pwm controller of the motor
float speed_pid_controller(){
  float p_speed = 60;
  float i_speed = 4;
  float d_speed = 0;
  float speed_diff = desired_speed - x_d;
  speed_integral += speed_diff;
  if(desired_speed <= windeup_threshold){
    speed_integral = 0;
  }
  if(speed_integral >= integral_max_threshold){
    speed_integral = integral_max_threshold;
  }
  if(input_timeout >=  max_input_timeout){
    speed_integral = 0;
  } 

  float motor_control = p_speed * speed_diff + i_speed * speed_integral;
  return motor_control;
}

//ros callback methods
void steering_cb( const std_msgs::Float64& cmd_msg){
  float tmp = - cmd_msg.data;
  tmp = tmp * 90/ (3.1416/2.0); 
  int out = (int) tmp;
  if(out > 90){
    out = 90;
  }else if(out < -90){
    out = -90;
  }
  out = 90 + out;
  steering.write(out); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

//callback from ros for desired travel speed
void speed_cb( const std_msgs::Float64& cmd_msg){
   desired_speed = cmd_msg.data;
   input_timeout = 0;
}

//take the computed motor_control and check for max_values and if input_timeout is to big
void control_motor_esc(float motor_control){
  if(motor_control > max_forward_pwm){
    motor_control = max_forward_pwm;
  }
  else if(motor_control < -max_backward_pwm){
    motor_control = -max_backward_pwm;
  }
  if(input_timeout >=  max_input_timeout){
    motor_control = 0;
  } 
  motor.write(90 - motor_control); //set servo angle, should be from 0-180  
}

void loop()
{
  nh.spinOnce();
  
  //measure computation time to set delay more precisely
  //unsigned long StartTime = micros();
  
  distance_traveled = compute_distance();
  add_value_to_array(distance_traveled);
  x_d = compute_speed();
  float motor_control = speed_pid_controller();
  control_motor_esc(motor_control);

  speed_msg.data = x_d;
  pub_speed.publish(&speed_msg);
  dist_msg.data = distance_traveled;
  pub_dist.publish(&dist_msg);
  
  //increase input_timeout for speed 
  input_timeout = input_timeout + 1;
  delay(50);    // run code every 50ms or 20 times per second
}

