#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int8

class Statemachine():
    def __init__(self):
        self.state = 1 # 0 = joystick, 1 = pwfl,...

        #acceleration | speed
        self.speed = 0.0
        self.steering = 0.0

        #init ros_node
        rospy.init_node('statemachine', anonymous=True)

        #init publisher
        self.ack_speed = rospy.Publisher("/bobcat/statemachine_speed/command", Float64, queue_size=1)
        self.ack_steer = rospy.Publisher("/bobcat/statemachine_steer/command", Float64, queue_size=1)
        self.pub_state = rospy.Publisher("/bobcat/statemachine/state", Int8, queue_size=1)

        #Joystick data
        rospy.Subscriber("bobcat/joy_steer/command", Float64, self.callback_joy_steer)
        rospy.Subscriber("bobcat/joy_speed/command", Float64, self.callback_joy_speed)
        #Pfwl data
        rospy.Subscriber("bobcat/pfwl_steer/command", Float64, self.callback_pfwl_steer)
        rospy.Subscriber("bobcat/pfwl_speed/command", Float64, self.callback_pfwl_speed)

    def callback_pfwl_steer(self, steer):
        if self.state != 1:
            return
        self.set_steering(steer.data)

    def callback_pfwl_speed(self, speed):
        if self.state != 1:
            return
        self.set_speed(speed.data)

    def callback_joy_steer(self, steer):
        if self.state != 0:
            return
        self.set_steering(steer.data)

    def callback_joy_speed(self, speed):
        if self.state != 0:
            return
        self.set_speed(speed.data)

    def set_speed(self, new_speed):
        self.speed = new_speed

    def set_steering(self, new_steering):
        self.steering = new_steering

    def publish(self):
        self.ack_speed.publish(Float64(self.speed))
        self.ack_steer.publish(Float64(self.steering))
        self.pub_state.publish(Int8(self.state))

def main():
    statemachine = Statemachine()
    rospy.loginfo("Statemachine started")
    rate = rospy.Rate(100) # 100hz
    #i = 0
    while not rospy.is_shutdown():
        statemachine.publish()
        #report decisions
        #if i%5:
        #    statemachine.printLog()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
