#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from bobcat_cone_classificator.msg import Obstacles_ext as Obst
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class Goal_point_calculator():

	def __init__(self):
		rospy.init_node('GoalPointCalculator', anonymous=True)
		print("init")
		#init subscriber
		rospy.Subscriber("/bobcat/classified_cones", Obst, self.callback_cones)
		#init publisher
		self.pubGoalPose = rospy.Publisher("/bobcat/goalPose", Point, queue_size=1)
		
		#just for testing		
		self.pubAckermannSpeed = rospy.Publisher("/bobcat/ackermann_speed/command", Float64, queue_size=1)
		self.pubAckermannSteer = rospy.Publisher("/bobcat/ackermann_steer/command", Float64, queue_size=1)


	def callback_cones(self, data):
		left_poles = data.left_cones
		right_poles = data.right_cones
		print(data)
		if len(left_poles) <= 0 or len(right_poles) <= 0:
			return
		indx_r = 0
		indx_l = 0
		smallest = 1000
		for i in range(0, len(right_poles)):
			dist = right_poles[i].x
			if(dist < smallest):
				smallest = dist
				indx_r = i
		smallest = 1000
		for i in range(0, len(left_poles)):
			dist = left_poles[i].x
			if(dist < smallest):
				smallest = dist
				indx_l = i
		print("left_pole", left_poles[indx_l])
		self.computeGoalPoint(left_poles[indx_l], right_poles[indx_r])



	def computeGoalPoint(self, left_pole, right_pole):
		x = (left_pole.x  + right_pole.x)/2.0			
		y = (left_pole.y  + right_pole.y)/2.0
		
		newPoint = Point(x,y,0)
		self.pubGoalPose.publish(newPoint)
		self.computeConotrolValues(x,y)

	def computeControlValues(self, x ,y):
		dist = math.sqrt(x**2 + y**2)
		speed = p_speed * dist
		steer = p_steer * y
		self.pubAckermannSpeed.publish(speed)
		self.pubAckermannSteer.publish(steer)


if __name__ == '__main__':
	try: 
		pubMpc = Goal_point_calculator()
    		rospy.loginfo("GoalPointCalculator started")
   		rospy.spin()
	except rospy.ROSInterruptException:
		pass


