#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from obstacle_detector import Obstacles

class OccupancyGrid():
     def __init__(self):
	self.subscribe()
      	#init ros_node
        rospy.init_node('occupancy_grd', anonymous=True)

     def subscribe(self):
        # init subscriber
        rospy.Subscriber("/raw_obstacles", Obstacles, self.callback_obstackles)

     def callback_obstackles(self, data):
	print(data)


def main():
    acc = OccupancyGrid()
    rospy.loginfo("Occupancy_Grid started")
    rospy.spin()  

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
