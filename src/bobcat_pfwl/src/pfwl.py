#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class PFWL():
    def __init__(self):
        #acceleration | speed
        self.speed = 10.0
        #self.speed = 0.0
        #value between -1.0 and 1.0 | straight 0.0
        self.steering = 0.0

        #CONSTANTS from model
        self.alpha = 9.0

        #scans
        self.left_dist = 1.0
        self.middle_dist = 1.0
        self.right_dist = 1.0

        #init ros_node
        rospy.init_node('pfwl', anonymous=True)

        #init publisher
        self.ack_speed = rospy.Publisher("/bobcat/pfwl_speed/command", Float64, queue_size=1)
        self.ack_steer = rospy.Publisher("/bobcat/pfwl_steer/command", Float64, queue_size=1)

        rospy.Subscriber("/scan",LaserScan,self.callback)

    def callback(self,scan):
        self.scan = scan

        left = 1081/5*2
        leftleft = 1081/4
        right = 1081/5*3
        rightright = 1081*3/4
        middle = 1081/2

        #OLD CODE
        # self.left_dist = scan.ranges[right]
        # if self.left_dist == float('inf'):
        #     self.left_dist = 10.0
        # elif self.left_dist < 0.1:
        #     self.left_dist = 0.1
        # self.middle_dist = scan.ranges[middle]
        # if self.middle_dist == float('inf'):
        #     self.middle_dist = 10.0
        # elif self.middle_dist < 1.0:
        #     self.middle_dist = 1.0
        # self.right_dist = scan.ranges[left]
        # if self.right_dist == float('inf'):
        #     self.right_dist = 10.0
        # elif self.right_dist < 0.1:
        #     self.right_dist = 0.1

        rate = scan.ranges[leftleft]/scan.ranges[rightright]
        if rate < 1:
            self.steering = -(scan.angle_increment*leftleft+scan.angle_min)
            rospy.loginfo("left")
        else:
            self.steering = -(scan.angle_increment*rightright+scan.angle_min)
            rospy.loginfo("right")

        self.set_steering(self.steering)

        rospy.loginfo("Winkel (rad): "+str(self.steering))

    def set_speed(self, new_speed):
        self.speed = new_speed

    def set_steering(self, new_steering):
        self.steering = new_steering

    def decideSteering(self):
        rate = self.right_dist/self.left_dist
        if rate == 1.0:
            self.set_steering = 0.0
        if rate<1:
            diff = self.left_dist - self.right_dist
            self.steering = diff / self.alpha
            if self.steering > 1.5:
                self.steering = 1.5
        else:
            diff = self.right_dist - self.left_dist
            self.steering = diff / self.alpha * (-1)
            if self.steering < -1.5:
                self.steering = -1.5
        rospy.loginfo("steering: "+str(self.steering))

    def publish(self):
        self.ack_speed.publish(Float64(self.speed))
        self.ack_steer.publish(Float64(self.steering))

def main():
    pfwl = PFWL()
    rospy.loginfo("PFWL started")
    rate = rospy.Rate(20) # 100hz
    i = 0
    while not rospy.is_shutdown():
        #pfwl.subscribe()
        #pfwl.decideSteering()
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
