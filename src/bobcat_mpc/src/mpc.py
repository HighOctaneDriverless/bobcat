#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from scipy.optimize import minimize
from scipy import interpolate
from geometry_msgs.msg import Pose
from bobcat_mpc.msg import MPCControlVec

#Vehicle Parameter for rc-car
lf = 0.2    # distance front tire to COG
lr = 0.2    # distance back  tire to COG
lb = 0.3    # width of car
r  = 0.05   # radius wheels

max_speed = 30 	  # for now max output for ackermann_speed
max_long_dec = -1
max_long_acc = 1
max_lat_acc = 20  # 2g lateral acceleration
max_steering_angle = (30.0/180.0)*math.pi #

#mpc variables
dt = 0.05
N = 5
initState = [0,0,0.1,math.pi/2.0]
goalPoint = [0.2,0]
X0 = np.zeros(6*(N+1)) 
running = False


def get_bounds():
        bnds = ((None, None),(None, None),
                (0, max_speed),(None, None),
                (max_long_dec, max_long_acc),
                (-max_steering_angle, max_steering_angle))*(N + 1)
	return bnds

def constraint_fix_init_state(x):
        """constraints the first 4 values to not be changed as of initial state"""
        ceq = x[0:4] - initState
        return ceq
    

def constraint_vehicle_model( x):
        N = x.size/6 -1
        ceq = np.zeros(N*4)
        for i in range(0,N,1):
            current_State = x[i*6: (i+1)*6]
            ceq[i*4: (i+1)*4] = x[(i+1)*6: (i+1)*6 + 4] - compute_next_state(current_State)		
        #return an array of 0= Xnext_state - model(Xnow_state)	
        return ceq

def compute_next_state( current_state):
 	x,y,v,orient,acc,steer = current_state
        Xnext = np.zeros(4)
        beta = np.arctan((lr/(lf +lr)) * math.tan(steer))
        Xnext[0] = x + v * dt * math.sin(orient + beta)
        Xnext[1] = y + v * dt * math.cos(orient + beta)
        Xnext[3] = orient + (v*dt/lr) * math.sin(beta)
        Xnext[2] = v + acc * dt
        if(Xnext[2] > max_speed): Xnext[2] = max_speed
        return Xnext

def cost_func(X):
	x = X[-6]
	y = X[-5]    
	return math.fabs(np.sqrt((x - goalPoint[0])**2 + (y - goalPoint[1])**2))
   

def init_param():
	init_message = rospy.get_param("~message", "hello")
	rospy.loginfo(rospy.get_name() + "message param is %s", init_message)

def get_cons():
        cons =({'type': 'eq', 'fun': constraint_fix_init_state}, 
                    {'type': 'eq', 'fun': constraint_vehicle_model}) 
	return cons

def main():
	# Initialize the node and name it.
	rospy.init_node('bobcat_mpc', anonymous = True)
	#get param
	init_param()
	#init running param
	running = False
	# Go to the main loop.
 	rate = rospy.Rate(int(1.0/dt)) # 10hz    	
	while not rospy.is_shutdown():

		#set_initial_state(X0[0:4]) set this as soon as we get a vehicle speed measurement 
		start_time = time.time()		
		res = minimize(cost_func, X0, method ='SLSQP',  bounds = bnds, constraints= cons, options={'ftol': 1e-4, 'disp':False})     
		#print("--- %s seconds ---" % (time.time() - start_time))		
		myFormattedList = [ '%.2f' % elem for elem in res.x ]
		#print("res.x", myFormattedList)
		if running:
			acc = res.x[4::6]
			steer = res.x[5::6]
			x = np.insert(steer, np.arange(len(acc)), acc)
			#print("res.x", res.x)
			#print("mpc_control_vec", x)
			mpc_vec = MPCControlVec(N, dt, x)
			pubMPCControlVec.publish(mpc_vec)
			running = False
		else:
			acc = res.x[4::6]
			steer = res.x[5::6]
			x = np.insert(steer, np.arange(len(acc)), acc)
			#print("res.x", res.x)
			#print("mpc_control_vec", x)
			mpc_vec = MPCControlVec(N, dt, x)
			pubMPCControlVec.publish(mpc_vec)
			running = False
        	rate.sleep()

def callback_goal_pose(data):
	running = True
	goalPoint[0] = data.position.x
	goalPoint[1] = data.position.y

	
	
    
#init publisher
pubMPCControlVec = rospy.Publisher("/bobcat/mpc_control_vec", MPCControlVec, queue_size=1)

#init subscriber
rospy.Subscriber("/bobcat/goalPose", Pose, callback_goal_pose)


#init bnds
bnds = get_bounds()
cons = get_cons()



# Main function.
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

