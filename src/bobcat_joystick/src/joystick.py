#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
import math

class Ackermann():
    def __init__(self):
	self.subscribe()
        self.motor = 0.0     
        self.steering = 0.0

        #init ros_node
        rospy.init_node('joystick_control', anonymous=True)
        #init publisher
        self.pub_acc = rospy.Publisher("motor", Int8, queue_size=1)
        self.pub_steering = rospy.Publisher("steering", Int8, queue_size=1)

    def callback_controller(self,data):
	self.set_speed(data.axes[1])
        self.set_steering(data.axes[0])
        #rospy.loginfo("Joystick speed "+ str(int(self.motor)*100))

    def set_speed(self, new_speed):
        self.motor = new_speed

    def set_steering(self, new_steering):
        self.steering = new_steering

    def subscribe(self):
        # init subscriber
        rospy.Subscriber("joy", Joy, self.callback_controller)
 
    def publish(self):
	self.pub_acc.publish(Int8(self.motor*100))
        self.pub_steering.publish(Int8(self.steering*100))
        rospy.loginfo("Motor: "+str(int(self.motor*100))+"Steering: "+str(int(self.steering*100)))


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
