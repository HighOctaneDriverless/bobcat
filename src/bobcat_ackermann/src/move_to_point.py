#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from bobcat_ackermann.msg import CarPose
import tf
import math

class MoveToPoint():
    def __init__(self):
        rospy.init_node('MoveToPoint', anonymous=True)
        self.subscribe()
        self.newGoalPose = CarPose()
        self.startPose = CarPose()
        self.newPose = CarPose()
        
        self.maxSpeed = 2 #m/s
        self.kbeta_t = 1.5
        self.kalpha = 0.7
        self.moveToPointActive = False
        self.direction = 0
        self.lastRho = 100000
        self.firstCall = True
        
        self.pubAckermannSpeed = rospy.Publisher("/bobcat/ackermann_speed/command", Float64,
                                            queue_size=1)
        self.pubAckermannSteer = rospy.Publisher("/bobcat/ackermann_steer/command", Float64,
                                             queue_size=1)
    
    def subscribe(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_state)
        #rospy.Subscriber("/bobcat/drive_to_point", Pose, self.callback_goal_pose)
        rospy.Subscriber("/bobcat/drive_to_point", CarPose, self.callback_goal_carpose)
  
    def callback_state(self, data):        
        pose  = data.pose[-1]                
        self.newPose.x = pose.position.x
        self.newPose.y = pose.position.y
        self.newPose.yaw = self.transform_quaternion_euler(pose)[2]
        
        if(self.moveToPointActive):
            pub = self.move_to_point()
            self.pubAckermannSpeed.publish(Float64(pub[0]))
            self.pubAckermannSteer.publish(Float64(pub[1]))
        else:
            self.pubAckermannSpeed.publish(Float64(0))
            self.pubAckermannSteer.publish(Float64(0))
 
    def callback_goal_carpose(self, data):
        #tfPose = tf.fromMsg(data)
        self.newGoalPose = data
        tmpStartPose = CarPose()
        tmpStartPose.x = self.newPose.x
        tmpStartPose.y = self.newPose.y
        tmpStartPose.yaw = self.newPose.yaw
        self.startPose = tmpStartPose
        self.moveToPointActive = True
        self.firstCall = True
        self.direction = 0
        
#==============================================================================
#     def callback_goal_pose(self, data):
#         #tfPose = tf.fromMsg(data)
#         print "callback goal pose"
#         pose = CarPose()
#         pose.x = data.position.x
#         pose.y = data.position.y
#         pose.yaw = self.transform_quaternion_euler(data)[2]
#         print pose
#         self.newGoalPose = pose
#         tmpStartPose = CarPose()
#         tmpStartPose.x = self.newPose.x
#         tmpStartPose.y = self.newPose.y
#         tmpStartPose.yaw = self.newPose.yaw
#         self.startPose = tmpStartPose
#         self.moveToPointActive = True
#         self.direction = 0
#         self.lastRho = 100000
#==============================================================================
    
    def move_to_point(self):

        tmpNewCarPose = self.newPose
                
        tmpStartPose = CarPose()
        tmpStartPose.x = self.startPose.x
        tmpStartPose.y = self.startPose.y
        tmpStartPose.yaw = self.startPose.yaw


        
        #calculate Car offset 
        tmpPoseDiff = CarPose() # x,y,yaw
        tmpPoseDiff.x = tmpNewCarPose.x - tmpStartPose.x
        tmpPoseDiff.y = tmpNewCarPose.y - tmpStartPose.y
        tmpPoseDiff.yaw = tmpNewCarPose.yaw - tmpStartPose.yaw
        
        
        #correct yaw to -pi/2 <-> pi/2
        if tmpPoseDiff.yaw > math.pi :
            tmpPoseDiff.yaw = tmpPoseDiff.yaw - math.pi * 2
        elif tmpPoseDiff.yaw < - math.pi:
            tmpPoseDiff.yaw = tmpPoseDiff.yaw + math.pi * 2
        
        #rotate coordinates for local pose
        ySin = tmpPoseDiff.y * math.sin(tmpPoseDiff.yaw)
        yCos = tmpPoseDiff.y * math.cos(tmpPoseDiff.yaw)
        xCos = tmpPoseDiff.x * math.cos(tmpPoseDiff.yaw)
        xSin = tmpPoseDiff.x * math.sin(tmpPoseDiff.yaw)
        
        tmp1 = ySin + xCos
        tmp2 = xSin + yCos
        print "local pose  x: " + str(tmp1) + "y: " + str(tmp2) + "yaw: " + str(tmpPoseDiff.yaw * 180/math.pi)
        print "difference goal yaw tmp yaw: " + str(self.newGoalPose.yaw - tmpPoseDiff.yaw)
        # calculate delta x and delta y
        deltaX = ySin + xCos - self.newGoalPose.x
        deltaY = xSin + yCos - self.newGoalPose.y
        
        
        #distance to goal
        rho = math.sqrt(deltaX**2 + deltaY**2)
        if(self.firstCall):
            self.firstCall = False
            self.lastRho = rho
#==============================================================================
#         print "rho : " + str(rho)
#         print "abs lastRho - rho: " + str(abs(self.lastRho - rho))
#==============================================================================
        if (abs(self.lastRho - rho) > 0.2):
            self.moveToPointActive = False
            print "missed point"
        
        if(self.lastRho > rho):            
            self.lastRho = rho

        
        speedOut = rho * self.maxSpeed  + 0.3
        if(speedOut > self.maxSpeed):
            speedOut = self.maxSpeed
        
        #print "tmpPoseDiff.yaw: " + str(tmpPoseDiff.yaw)
        #calculate steering 
        if self.direction == 0: #first run
            beta = - math.atan2(- deltaY, - deltaX)
            alpha = - tmpPoseDiff.yaw - beta
            
            if(alpha > math.pi/2.0 or alpha < - math.pi/2.0):
                self.direction = -1
            else:
                self.direction = 1
        elif(self.direction == -1):
            beta = - math.atan2(deltaY, deltaX)
            alpha = - tmpPoseDiff.yaw - beta
        else:
            beta = - math.atan2(- deltaY, - deltaX)
            alpha = - tmpPoseDiff.yaw - beta
        
        if alpha > math.pi/2.0:
            alpha = math.pi/2.0
        if alpha < - math.pi /2.0:
            alpha = - math.pi /2.0
        
        beta_ = beta + self.newGoalPose.yaw
        gamma = self.direction * (-self.kbeta_t * beta_ + self.kalpha * alpha)
#==============================================================================
#         print "alpha : " + str(alpha)
#         print "beta : " + str(beta)
#         print "beta_ : " + str(beta_)        
#         print "gamma : " + str(gamma)        
#      
#==============================================================================
        steerOut = gamma 
        speedOut = speedOut *self.direction
#==============================================================================
#         print "steerOut : " + str(steerOut)        
#         print "speedOut :" + str(speedOut)
#         
#==============================================================================
        if rho < 0.06:
            print "hit point"
            self.moveToPointActive = False
            
        return [speedOut, steerOut]
        


        

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

def main():

    moveToPoint = MoveToPoint()
    rospy.loginfo("MoveToPoint started")
    rospy.spin()   
#==============================================================================
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#             
#         rate.sleep()
#                 
#==============================================================================

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
