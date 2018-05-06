#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys
import math
from obstacle_detector.msg import Obstacles as obs
from bobcat_cone_classificator.msg import Obstacles_ext as obs_ext

class Camera_view():
	def __init__(self):
		#init ros_node
        	rospy.init_node('pfwl', anonymous=True)

		self.timedelta = 0
		rospy.Subscriber("/camera/rgb/image_raw",Image,self.callbackRGB)
		self.image = np.zeros((480,640,3))
		rospy.Subscriber("/raw_obstacles",obs,self.callbackObstacle)
		#rospy.Subscriber("/camera/depth/image",Image,self.callbackDepth)
		self.publish_obs_ext = rospy.Publisher("/bobcat/classified_cones", obs_ext, queue_size=1)
		
		self.temp = obs_ext()

		self.img_sliced = np.ones((1,640,3))
		#images
		#self.rgb_slice = np.zeros((40,640,3))
		#self.d_slice = np.zeros((40,640,1))
		
		self.x_vals = []
		#canny parameters
		self.ratio = 3
		self.kernel_size = 3
		self.threshold = 50 # between 0 and 100

		self.color_left = 120
		self.color_right = 90

	def callbackObstacle(self,obstacles):
		self.temp = obs_ext()
		print("obstacles", len(obstacles.circles))
		objects = obstacles.circles
		self.x_vals = []		
		for i in range(0,len(objects)):
			rad = math.atan(objects[i].center.x/objects[i].center.y)
			degree = rad*180/math.pi
			if(degree>0):
				self.x_vals.append(295/29*abs(61-degree))
			else:
				self.x_vals.append((640-295)/29*(90-abs(degree))+295)
			#print(self.image_sliced)
			#color = self.image_sliced[0,x_val,:]
			#print('color', color) 
			#print("pixel", self.x_vals[i])			
			#print('angel '+str(i),degree)
			#print('rad' +str(i),rad)
		self.color(objects)
		

	def callbackRGB(self,image):
		np_arr = np.fromstring(image.data, np.uint8)
		#print(str(np_arr.shape))
		self.image = np_arr.reshape((480,640,3))
		self.image = self.image[...,::-1]
		self.img_sliced = self.image[240:241,:,:]
		#print(image_sliced)
		
		#rgb_slice = np.repeat(self.img_sliced,40,axis=0)
		#self.canny()
		#print(self.img_sliced.shape)
		#cv2.imshow('Image',rgb_slice)
		#cv2.waitKey(1)

	def color(self,objects):
		print(self.x_vals)
		for i in range(0,len(self.x_vals)):
			#print('object ',i)
			#print(self.img_sliced[0,int(self.x_vals[i])])
			color = self.img_sliced[0,int(self.x_vals[i]),:]
			#color = np.uint8(color)/255.0
			color_np = np.array(color)
			color_np = np.expand_dims(color_np,axis=0)
			color_np = np.expand_dims(color_np,axis=0)
			l_or_r = self.classify_color(color_np)
			if l_or_r == 1:
				self.temp.left_cones.append(objects[i])
			elif l_or_r == 2:
				self.temp.right_cones.append(objects[i])
			#print('color', color)
		print("pub",self.temp)
		self.publish_obs_ext.publish(self.temp)
		rgb_slice = np.repeat(self.img_sliced,40,axis=0)
		#self.canny()
		#print(self.img_sliced.shape)
		cv2.imshow('Image',rgb_slice)
		cv2.waitKey(1)

	def classify_color(self,color):
		#print("shape",self.img_sliced.shape)
		hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
		#print("hsv",hsv[0,0,0])
		h = hsv[0,0,0]
		if abs(h-self.color_left)<10:
			#print("left")
			return 1
		elif abs(h-self.color_right)<15:
			#print("right")			
			return 2

	def callbackDepth(self,image):
		np_arr = np.fromstring(image.data, np.float32)
		#print(str(np_arr.shape))
		rgbd = np_arr.reshape((480,640,1))
		#print(str(rgbd.shape))		
		#print(rgbd.shape)
		#print(rgbd[:,:,0:3].shape)		
		#cv2.imshow('test',rgbd[:,:,0:3])
		rgbd_slice = rgbd[240:241,:,:]
		#print(str(rgbd_slice.shape))
		d_slice = np.repeat(rgbd_slice,40,axis=0)
		print(rgbd_slice[0:1,320:321,0:1])
		#print(str(slice_image.shape))
		slice_dist=np.zeros((2,640))
		slice_range = np.arange(640)
		slice_dist[0,] = rgbd_slice[0,:,0]
		slice_dist[1,] = slice_range
		slice_dist = np.nan_to_num(slice_dist)
		print(slice_dist.shape)
		print("test",rgbd_slice[0,:,0].shape)
		self.kernel(slice_dist)
		cv2.imshow('test',d_slice)
		cv2.waitKey(1)

	def show(self):
		#cv2.imshow('RGB',self.rgb_slice)
		#cv2.imshow('Depth',self.d_slice)
		cv2.waitKey(1)
		#cv2.imshow(self.image)
		#cv2.imwrite("")
		
	
	def write(self):
		try:
			filename = "/home/nvidia/catkin_ws/src/bobcat/src/bobcat_pfwl/pics/image_t"+str(self.timedelta)+".png"
			cv2.imwrite(filename,self.image)
			self.timedelta = self.timedelta + 1
			print("file saved")
		except: 
			print("error",sys.exc_info()[0])


def main():
	cam = Camera_view()
	rospy.loginfo("cam infos started")
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		#cam.write()
		#cam.show()
		#cam.color()
		#cam.write()
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
