#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
import math
import tf

class Ackermann():
    def __init__(self):
        self.subscribe()
        #acceleration | speed
        self.desired_speed = Float64(0.0)
        self.measured_speed = 0.0
        self.measured_speed_meadian = 0.0
        self.median_filter = np.zeros([5])
        self.speed_out = Float64(0.0)
        #value between -1.0 and 1.0 | straight 0.0
        self.steering = 0.0
        self.steeringRight = 0.0
        self.steeringLeft = 0.0


        #CONSTANTS from model
        self.L = 2.6     #wheelbase
        self.lw = 1.301  #width

        #init ros_node
        rospy.init_node('ackermann_control', anonymous=True)
        #init publisher
        self.pub_acc_left = rospy.Publisher("/bobcat/left_rear_wheel_effort_controller/command", Float64, queue_size=1)
        self.pub_acc_right = rospy.Publisher("/bobcat/right_rear_wheel_effort_controller/command", Float64, queue_size=1)
        self.pub_steering_left = rospy.Publisher("/bobcat/left_steering_hinge_position_controller/command", Float64,
                                            queue_size=1)
        self.pub_steering_right = rospy.Publisher("/bobcat/right_steering_hinge_position_controller/command", Float64,
                                             queue_size=1)

    def callback_steering(self, data):
        desiredSteeringAngle = data.data   
	maxCarSteerAngle = 30*math.pi/180  
	maxSteerAngle = self.L/((self.L/maxCarSteerAngle) + self.lw/2.0)
        
        if(desiredSteeringAngle > 0):
	    if(desiredSteeringAngle > maxSteerAngle):
		desiredSteeringAngle = maxSteerAngle            
            self.steeringRight = self.L/((self.L/desiredSteeringAngle) + self.lw/2.0)         
            self.steeringLeft = self.L/((self.L/desiredSteeringAngle) - self.lw/2.0)
        elif(desiredSteeringAngle < 0):
            if(desiredSteeringAngle < -maxSteerAngle):
		desiredSteeringAngle = -maxSteerAngle            
            self.steeringRight = self.L/((self.L/desiredSteeringAngle) + self.lw/2.0)         
            self.steeringLeft = self.L/((self.L/desiredSteeringAngle) - self.lw/2.0)
        else:
            self.steeringRight = 0
            self.steeringLeft = 0
            

    def callback_speed(self, data):
        self.set_speed(data.data)
        
        
    def transform_quaternion_euler(self, pose):
        #type(pose) = geometry_msgs.msg.Pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]                
        return euler 

    def callback_state(self, data):
        orient = self.transform_quaternion_euler(data.pose[-1])[2]
        x = data.twist[-1].linear.x
        y = data.twist[-1].linear.y
        
        orient = orient % (2*math.pi)
        if(orient <= math.pi/2 or orient >= (math.pi + math.pi /2)):
            if(x >= 0.0): #drives forward
                direction = 1
            else: #drives backwards
                direction = -1
        else:
            if(x >= 0.0): # drives backwards
                direction = -1
            else: # drives forwards
                direction = 1
            
        self.measured_speed = math.sqrt(x**2 + y**2) * direction
  
        self.median_filter = np.roll(self.median_filter, -1)
        self.median_filter[-1] = self.measured_speed
        #print("%.2f" % a)
        rospy.loginfo("speed_x: "+ str("%.2f" % x) + "  speed_y: " + str("%.2f" % y) + "  orientation: " + str("%.2f" % orient) + "  direction: " + str(direction))


    def set_speed(self, new_speed):
        if new_speed > 2:
                new_speed = 2
        if new_speed < -2:
                new_speed = -2
        self.desired_speed.data = new_speed

    def set_steering(self, new_steering):
        self.steering = new_steering

    def subscribe(self):
        # init subscriber
        rospy.Subscriber("/bobcat/ackermann_steer/command", Float64, self.callback_steering)
        rospy.Subscriber("/bobcat/ackermann_speed/command", Float64, self.callback_speed)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_state)
        #rospy.Subscriber("joy", Joy, self.callback_controller)
        #perhaps important. has to be checked
        #rospy.spin()

    def pid(self):
        p = 600 
 
        diff =  self.desired_speed.data - np.median(self.median_filter)
        self.speed_out = diff * p
	if(self.speed_out > 1000):
		self.speed_out = 1000	
	if(self.speed_out < -1000):
		self.speed_out = -1000	
        #rospy.loginfo("desired_speed "+ str(self.desired_speed.data) + "  measured_speed" + str(np.median(self.median_filter)) + "  output: " + str(self.speed_out))

    def publish(self):
        self.pid()
        self.pub_acc_left.publish(self.speed_out)
        self.pub_acc_right.publish(self.speed_out)
        self.pub_steering_left.publish(self.steeringLeft)
        self.pub_steering_right.publish(self.steeringRight)
        #rospy.loginfo("Speed: "+str(self.speed_out)+"Steer: "+str(self.steering))


def main():
    ackerm = Ackermann()
    rospy.loginfo("Ackermann started")
    rate = rospy.Rate(20) # 
    while not rospy.is_shutdown():

        ackerm.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
