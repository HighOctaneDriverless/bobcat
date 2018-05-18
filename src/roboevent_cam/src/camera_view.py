#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys
import math
from skimage.transform import resize 
from skimage import img_as_ubyte, util, color, io
#from obstacle_detector.msg import Obstacles as obs
#from bobcat_cone_classificator.msg import Obstacles_ext as obs_ext
#from geometry_msgs.msg import Point

class Camera_view():
	def __init__(self):
		#init ros_node
        	rospy.init_node('roboevent_cam', anonymous=True)
		self.iteration = 0

		rospy.Subscriber("/camera/rgb/image_raw",Image,self.callbackRGB)
		self.img_to_save = False
		self.publish_img = rospy.Publisher("/bobcat/roboevent_cam", Image, queue_size=1)
		
	def callbackRGB(self,image):
		np_arr = np.fromstring(image.data, np.uint8)
		#print(str(np_arr.shape))
		#self.image = np_arr.reshape((480,640,3))
		img_reshaped = np_arr.reshape((480,640,3))
		

		#BGR2RGB
		img_reshaped = img_reshaped[:,:,::-1]

		#resize to (240,320,3)
		img_resized = resize(img_reshaped,(240,320))
		img_resized = img_as_ubyte(img_resized)
		

		#crop to (96,320,3)
		img_cropped = img_resized[143:239,0:320,:3]

		#noise
		img_cropped = util.random_noise(img_cropped,var=0)
		img_cropped = color.rgb2gray(img_cropped)		
		img_cropped = img_as_ubyte(img_cropped)
		
		#img_cropped = np.expand_dims(img_cropped, axis=2)
		#print("shape",img_cropped.shape)


		#print("img_to save",self.img_to_save)
		if self.img_to_save == True:
			#print("test")
			self.savePicture(img_cropped)	

		
		self.publish(img_cropped)
		#self.show(img_cropped)
	
	def show(self,img):
		#if img.shape[2] != 3:
		#	img = img[:,:]
		cv2.imshow('RGB',img)
		#cv2.imshow('Depth',self.d_slice)
		cv2.waitKey(1)
		#cv2.imshow(self.image)
		#cv2.imwrite("")

	def publish(self, img_cropped):
		img_pub = Image()		
		self.show(img_cropped)
		img_pub.data = img_cropped.flatten().tolist()
		self.publish_img.publish(img_pub)

	def savePicture(self, img):		
		self.iteration = self.iteration +1
		if self.iteration %100 == 0:
			print("saving")
			io.imsave("/home/nvidia/catkin_ws/src/bobcat/src/roboevent_cam/src/pics/"+str(self.iteration/100)+".png",img)
			#cv2.imwrite("/home/nvidia/catkin_ws/src/bobcat/src/roboevent_cam/src/pics/"+str(self.iteration)+".png",img)
		#self.img_to_save = False
		

def main():
	cam = Camera_view()
	rospy.loginfo("cam infos started")
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		#cam.publish()
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
