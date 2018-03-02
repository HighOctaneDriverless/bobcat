#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Joy
import math

class Ackermann():
    def __init__(self):
        self.subscribe()
        #acceleration | speed
        self.desired_speed = Float64(0.0)
        self.measured_speed = 0.0
        self.speed_out = Float64(0.0)
        #value between -1.0 and 1.0 | straight 0.0
        self.steering = 0.0
        self.steering_left = 0.0
        self.steering_right = 0.0


        #CONSTANTS from model
        self.L = 2.0   #Radstand
        self.Ba = 1.0  #Achsschenkelbolzenabstand

        #init ros_node
        rospy.init_node('ackermann_control', anonymous=True)
        #init publisher
        self.pub_acc_left = rospy.Publisher("/bobcat/left_rear_wheel_effort_controller/command", Float64, queue_size=1)
        self.pub_acc_right = rospy.Publisher("/bobcat/right_rear_wheel_effort_controller/command", Float64, queue_size=1)
        self.pub_steering_left = rospy.Publisher("/bobcat/left_steering_hinge_position_controller/command", Float64,
                                            queue_size=1)
        self.pub_steering_right = rospy.Publisher("/bobcat/right_steering_hinge_position_controller/command", Float64,
                                             queue_size=1)

    def callback_steering(self, data):
        self.set_steering(data.data)

    def callback_speed(self, data):
        self.set_speed(data.data)

    def callback_state(self, data):
        x = data.twist[-1].linear.x
        y = data.twist[-1].linear.y
        if x+ y > 0.0:
            self.measured_speed = math.sqrt(x**2 + y**2)
        else:
            self.measured_speed = - math.sqrt(x**2 + y**2)

    def callback_controller(self,data):
        self.set_speed(data.axes[1]/2)
        self.set_steering(data.axes[0])
        rospy.loginfo("Joystick speed "+ str(data.axes[1]/2))


    def set_speed(self, new_speed):
        self.desired_speed.data = new_speed / 10

    def set_steering(self, new_steering):
        self.steering = new_steering

    def subscribe(self):
        # init subscriber
        rospy.Subscriber("/bobcat/ackermann_steer/command", Float64, self.callback_steering)
        rospy.Subscriber("/bobcat/ackermann_speed/command", Float64, self.callback_speed)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_state)
        #rospy.Subscriber("joy", Joy, self.callback_controller)
        #perhaps important. has to be checked
        #rospy.spin()

    def pid(self):
        p = 2.0 # 3 as initial guess :D i and d missing for now needs to be determined how much that is needed
        #print "desired_speed : " + repr(self.desired_speed) + "   measured_speed: " + repr(self.measured_speed)
        #print "measured_speed: "
        #print type(self.measured_speed)
        #print "desired_speed: "
        #print type(self.desired_speed.data)
        diff =  self.desired_speed.data - self.measured_speed

        self.speed_out = diff * p
        if (self.speed_out > 0.6):
            self.speed_out = 0.6
        elif (self.speed_out < -0.6):
            self.speed_out = -0.6

    def publish(self):
        self.pid()
        self.pub_acc_left.publish(self.speed_out)
        self.pub_acc_right.publish(self.speed_out)
        self.pub_steering_left.publish(self.steering)
        self.pub_steering_right.publish(self.steering)
        #rospy.loginfo("Speed: "+str(self.speed_out)+"Steer: "+str(self.steering))


def main():
    ackerm = Ackermann()
    rospy.loginfo("Ackermann started")
    rate = rospy.Rate(100) # 
    while not rospy.is_shutdown():

        ackerm.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
