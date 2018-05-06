#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from bobcat_cone_classificator.msg import Obstacles_ext as Obst
from geometry_msgs.msg import Point

class Goal_point_calculator():

	def __init__(self):
		rospy.init_node('GoalPointCalculator', anonymous=True)
		#init subscriber
		rospy.Subscriber("/bobcat/classified_cones", Obst, self.callback_cones)
		#init publisher
		pubGoalPose = rospy.Publisher("/bobcat/goalPose", Pose, queue_size=1)


	def callback_cones(self, data):
		left_poles = data.left_poles
		right_poles = data.right_poles
		indx_r = 0
		indx_l = 0
		smallest = 1000
		for i in range(0, len(right_poles)):
			dist = right_poles[i].center.x
			if(dist < smallest):
				smallest = dist
				indx_r = i
		smallest = 1000
		for i in range(0, len(left_poles)):
			dist = left_poles[i].center.x
			if(dist < smallest):
				smallest = dist
				indx_l = i

		self.computeGoalPoint(left_pole[indx_l], right_pole[indx_r])


	def computeGoalPoint(self, left_pole, right_pole):
		x = (left_pole.x	+ right_pole.x)/2.0			
		y = (left_pole.y - right_pole.y)/2.0
		
		newPoint = Point(x,y,0)
		pubGoalPose.publish(newPoint)




if __name__ == '__main__':
	try: 
		pubMpc = Goal_point_calculator()
    		rospy.loginfo("GoalPointCalculator started")
   		rospy.spin()
	except rospy.ROSInterruptException:
		pass



