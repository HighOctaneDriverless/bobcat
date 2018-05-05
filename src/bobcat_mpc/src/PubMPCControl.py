#!/usr/bin/env python

import rospy
import time
import numpy as np
from bobcat_mpc.msg import MPCControlVec
from std_msgs.msg import Float64


class PubMpcControl():

	def __init__(self):
		self.counter = 0
		self.N = 0
		self.dt = 0.05
		self.control_vec = [0,0]

	def callback_mpc_control_vec(self, data):
		self.N = data.N
		self.dt = data.dt
		self.control_vec = data.control_vec
		self.counter = 0
		#print("data", data)
			

	def main(self):
		# Initialize the node and name it.
		rospy.init_node('bobcat_pub_mpc_control', anonymous = True)
		rospy.loginfo("PubMPCControl started")

		#init publisher
		pubAckermannSpeed = rospy.Publisher("/bobcat/ackermann_speed/command", Float64, queue_size=1)
		pubAckermannSteer = rospy.Publisher("/bobcat/ackermann_steer/command", Float64, queue_size=1)

		#init subscriber
		rospy.Subscriber("/bobcat/mpc_control_vec", MPCControlVec, self.callback_mpc_control_vec)


	    	rate = rospy.Rate(20) # 20hz    	
	    	while not rospy.is_shutdown():
			if(self.counter < self.N):
				pubAckermannSpeed.publish(self.control_vec[self.counter*2 +0])
				pubAckermannSteer.publish(self.control_vec[self.counter*2 +1])				
				self.counter += 1
			else:
				pubAckermannSpeed.publish(0)
				pubAckermannSteer.publish(0)				
		
			rate.sleep()



	

	

if __name__ == '__main__':
	try: 
		pubMpc = PubMpcControl()
		pubMpc.main()
	except rospy.ROSInterruptException:
		pass
