#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class PFWL():
    def __init__(self):
        #acceleration | speed
        self.speed = 0.3
        #value between -1.0 and 1.0 | straight 0.0
        self.steering = 0.0

        #CONSTANTS from model
        self.alpha = 30.0

        #scans
        self.left_dist = 1.0
        self.middle_dist = 1.0
        self.right_dist = 1.0

        #init ros_node
        rospy.init_node('pfwl', anonymous=True)

        #init publisher
        self.ack_speed = rospy.Publisher("/bobcat/ackermann_speed/command", Float64, queue_size=1)
        self.ack_steer = rospy.Publisher("/bobcat/ackermann_steer/command", Float64, queue_size=1)

    def callback(self,scan):
        self.scan = scan
        self.left_dist = scan.ranges[1080/5*3]
        if self.left_dist == float('inf'):
            self.left_dist = 10.0
        elif self.left_dist < 0.1:
            self.left_dist = 0.1
        self.middle_dist = scan.ranges[1080/2]
        if self.middle_dist == float('inf'):
            self.middle_dist = 10.0
        elif self.middle_dist < 1.0:
            self.middle_dist = 1.0
        self.right_dist = scan.ranges[1080/5*2]
        if self.right_dist == float('inf'):
            self.right_dist = 10.0
        elif self.right_dist < 0.1:
            self.right_dist = 0.1
        #rospy.loginfo((len(scan.ranges), min(scan.ranges)))
        #rospy.loginfo("left "+str(scan.ranges[1080/2])+" array number: "+str(1081/4))
        rospy.loginfo("left: "+str(self.left_dist)+" right: "+ str(self.right_dist) + "s_L: " + str(1080/5*2)+ " s_R: "+str(1080/5*3))

    def set_speed(self, new_speed):
        self.speed = new_speed

    def set_steering(self, new_steering):
        self.steering = new_steering

    def decideSteering(self):
        rate = self.right_dist/self.left_dist
        if rate == 0.0:
            rate = 0.1
        if self.left_dist < self.right_dist:
            self.steering = rate / self.alpha * (-1)
            #self.steering = (self.right_dist - self.left_dist)/self.alpha
        else:
            rate = 1/rate
            self.steering = rate / self.alpha
            #self.steering = (self.left_dist-self.right_dist)/self.alpha * (-1)

        rospy.loginfo("steering: "+str(self.steering))

    def subscribe(self):
        # init subscriber
        rospy.Subscriber("/scan",LaserScan,self.callback)
        #perhaps important. has to be checked
        #rospy.spin()

    def publish(self):
        self.ack_speed.publish(self.speed)
        self.ack_steer.publish(self.steering)

def main():
    pfwl = PFWL()
    rospy.loginfo("PFWL started")
    rate = rospy.Rate(10) # 100hz
    i = 0
    while not rospy.is_shutdown():
        pfwl.subscribe()
        pfwl.decideSteering()
        pfwl.publish()
        #report decisions
        if i%5:
            pfwl.printLog()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
