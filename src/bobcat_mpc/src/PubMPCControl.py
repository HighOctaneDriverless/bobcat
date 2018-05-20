#!/usr/bin/env python

import rospy
import time
import numpy as np
import serial
import struct
from bobcat_mpc.msg import MPCControlVec
from std_msgs.msg import Float64


class PubMpcControl():

	def __init__(self):
		self.counter = 0
		self.N = 0
		self.dt = 0.1
		self.control_vec = [0,0]
		#init publisher
		self.pubAckermannSpeed = rospy.Publisher("/bobcat/ackermann_speed/command", Float64, queue_size=1)
		self.pubAckermannSteer = rospy.Publisher("/bobcat/ackermann_steer/command", Float64, queue_size=1)

		#init subscriber
		rospy.Subscriber("/bobcat/mpc_control_vec", MPCControlVec, self.callback_mpc_control_vec)

		#init rosserial
		try:
			test = 0
			#self.ser = serial.Serial('/dev/ttyACM0', 9600)
		except ValueError:
			print(ValueError)

	def callback_mpc_control_vec(self, data):
		self.N = data.N
		self.dt = data.dt
		self.control_vec = data.control_vec
		self.counter = 0
		#print("data", data)
	
	def pubPWMSignalToArduino(self, speed, steer):
		pwm_speed = int(speed * 10 + 90)
		pwm_steer = int(steer * 180)
			
		self.ser.write(struct.pack('>BBB',pwm_speed, pwm_steer, 10))

		
	def main(self):
		# Initialize the node and name it.
		rospy.init_node('bobcat_pub_mpc_control', anonymous = True)
		rospy.loginfo("PubMPCControl started")




	    	rate = rospy.Rate(10) # 20hz    	
	    	while not rospy.is_shutdown():
			p_speed = 1.7
			if(self.counter < self.N):
				self.pubAckermannSpeed.publish(self.control_vec[self.counter*2 +0] * p_speed)
				self.pubAckermannSteer.publish(self.control_vec[self.counter*2 +1])		 		
				#self.fpubPWMSignalToArduino(self.control_vec[self.counter*2 + 0], self.control_vec[self.counter*2 + 1])
				self.counter += 1
			else:
				self.pubAckermannSpeed.publish(0)
				self.pubAckermannSteer.publish(0)	
				#self.pubPWMSignalToArduino( 0.0, 0.0)			
		
			rate.sleep()



	

if __name__ == '__main__':
	try: 
		pubMpc = PubMpcControl()
		pubMpc.main()
	except rospy.ROSInterruptException:
		pass
