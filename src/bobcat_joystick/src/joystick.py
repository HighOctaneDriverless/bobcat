#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import math

class Ackermann():
    def __init__(self):
        self.subscribe()
        self.speed = 0.0     
        self.steeringAngle = 0.0
	self.buttons = None

        #init ros_node
        rospy.init_node('joystick_control', anonymous=True)
        #init publisher
        self.pub_acc = rospy.Publisher("/bobcat/ackermann_speed/command", Float64, queue_size=1)
        self.pub_steering = rospy.Publisher("bobcat/ackermann_steer/command", Float64, queue_size=1)

    def callback_controller(self,data):
        print data
        self.set_speed(data.axes[1])
        self.set_steering(data.axes[2])
	self.buttons= data.buttons
        #rospy.loginfo("Joystick speed "+ str(int(self.motor)*100))

    def set_speed(self, new_speed):
        self.speed = new_speed

    def set_steering(self, new_steering):
        self.steeringAngle = new_steering

    def subscribe(self):
        # init subscriber
        rospy.Subscriber("joy", Joy, self.callback_controller)
 
    def publish(self):
	if self.buttons is not None and self.buttons[1]: # (b) Button pressed
		self.speed = 0
        	self.pub_acc.publish(Float64(0))
        ackermannSpeedOut = self.speed # outputting 10m/s max equals 36km/h
	if abs(self.speed) > 0.1:
        	self.pub_acc.publish(Float64(ackermannSpeedOut))
        ackermannSteerOut = self.steeringAngle * math.pi/6.0
	if abs(self.steeringAngle) > 0.1:
        	self.pub_steering.publish(Float64(ackermannSteerOut))
        rospy.loginfo("JoystickSpeed: "+str(round(ackermannSpeedOut, 3))+"   JoystickSteeringAngle: "+str(round(ackermannSteerOut, 3)))


def main():
    ackerm = Ackermann()
    rospy.loginfo("Joystick started")
    rate = rospy.Rate(100) # 
    while not rospy.is_shutdown():        
        ackerm.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
