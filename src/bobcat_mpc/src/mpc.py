#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from scipy.optimize import minimize
from scipy import interpolate
from geometry_msgs.msg import Point
from bobcat_mpc.msg import MPCControlVec

class MPC():


	def init_param(self):
		init_message = rospy.get_param("~message", "hello")
		rospy.loginfo(rospy.get_name() + "message param is %s", init_message)

		#Vehicle Parameter for rc-car
		self.lf = 0.2    # distance front tire to COG
		self.lr = 0.2    # distance back  tire to COG
		self.lb = 0.3    # width of car
		self.r  = 0.05   # radius wheels

		self.max_speed = 1 	# for now max output for ackermann_speed
		self.max_long_dec = -1
		self.max_long_acc = 1
		self.max_lat_acc = 20  # 2g lateral acceleration
		self.max_steering_angle = (30.0/180.0)*math.pi #

		#mpc variables
		self.dt = 0.1
		self.N = 6
		self.initState = [0,0,0.8, 0]
		self.goalPoint = [1.0,0.0]
		self.X0 = np.zeros(6*(self.N+1)) 
		self.running = False


	def get_bounds(self):
		bnds = ((None, None),(None, None),
		        (-self.max_speed, self.max_speed),(None, None),
		        (self.max_long_dec, self.max_long_acc),
		        (-self.max_steering_angle, self.max_steering_angle))*(self.N + 1)
		return bnds

	def constraint_fix_init_state(self, x):
		"""constraints the first 4 values to not be changed as of initial state"""
		ceq = x[0:4] - self.initState
		return ceq
	    

	def constraint_vehicle_model(self, x):
		N = x.size/6 -1
		ceq = np.zeros(N*4)
		for i in range(0,N,1):
		    current_State = x[i*6: (i+1)*6]
		    ceq[i*4: (i+1)*4] = x[(i+1)*6: (i+1)*6 + 4] - self.compute_next_state(current_State)		
		#return an array of 0= Xnext_state - model(Xnow_state)	
		return ceq

	def compute_next_state(self, current_state):
	 	x,y,v,orient,acc,steer = current_state
		Xnext = np.zeros(4)
		beta = np.arctan((self.lr/(self.lf + self.lr)) * math.tan(steer))
		Xnext[0] = x + v * self.dt * math.cos(orient + beta)
		Xnext[1] = y + v * self.dt * math.sin(orient + beta)
		Xnext[3] = orient + (v*self.dt/self.lr) * math.sin(beta)
		Xnext[2] = v + acc * self.dt
		if(Xnext[2] > self.max_speed): Xnext[2] = self.max_speed
		return Xnext

	def cost_func(self, X):
		x = X[-6]
		y = X[-5]   
		dist = math.fabs(np.sqrt((x - self.goalPoint[0])**2 + (y - self.goalPoint[1])**2))
	   	#print("dist", dist)
		return dist


	def get_cons(self):
		cons =({'type': 'eq', 'fun': self.constraint_fix_init_state}, 
		       {'type': 'eq', 'fun': self.constraint_vehicle_model}) 
		return cons

	def main(self):
		# Initialize the node and name it.
		rospy.init_node('bobcat_mpc', anonymous = True)

		#get param
		self.init_param()

		#init subscriber and publisher
		self.pubMPCControlVec = rospy.Publisher("/bobcat/mpc_control_vec", MPCControlVec, queue_size=1)
		rospy.Subscriber("/bobcat/goalPose", Point, self.callback_goal_point)


		# Go to the main loop.
	 	rate = rospy.Rate(int(1.0/self.dt)) # 10hz    
		#init bnds
		bnds = self.get_bounds()
		cons = self.get_cons()	
		while not rospy.is_shutdown():

			#set_initial_state(X0[0:4]) set this as soon as we get a vehicle speed measurement 
			start_time = time.time()		
			res = minimize(self.cost_func, self.X0, method ='SLSQP',  bounds = bnds, constraints= cons, options={'ftol': 1e-4, 'disp':False})     
			print("--- %s seconds ---" % (time.time() - start_time))		
			myFormattedList = [ '%.2f' % elem for elem in res.x ]
			#print("res.x", myFormattedList)
			if self.running:
				acc   = res.x[4::6]
				steer = res.x[5::6]
				x = np.insert(steer, np.arange(len(acc)), acc)
				#print("res.x", res.x)
				#print("mpc_control_vec", x)
				mpc_vec = MPCControlVec(self.N, self.dt, x)
				self.pubMPCControlVec.publish(mpc_vec)
				self.running = False

			rate.sleep()

	def callback_goal_point(self, data):
		#print("received data")
		self.running = True
		self.goalPoint[0] = data.x
		self.goalPoint[1] = data.y

	


# Main function.
if __name__ == '__main__':
    try:	
	mpc = MPC()
        mpc.main()
    except rospy.ROSInterruptException:
        pass

