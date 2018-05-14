#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys
import math
from skimage.transform import resize 
import skimage
#from obstacle_detector.msg import Obstacles as obs
#from bobcat_cone_classificator.msg import Obstacles_ext as obs_ext
#from geometry_msgs.msg import Point

class Camera_view():
	def __init__(self):
		#init ros_node
        	rospy.init_node('roboevent_cam', anonymous=True)

		rospy.Subscriber("/camera/rgb/image_raw",Image,self.callbackRGB)
		self.img_cropped = np.zeros((96,320,3))
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

		#crop to (96,320,3)
		self.img_cropped = img_resized[143:239,0:320,:3]

		self.show(self.img_cropped)	
			
	
	def show(self,img):
		cv2.imshow('RGB',img)
		#cv2.imshow('Depth',self.d_slice)
		cv2.waitKey(1)
		#cv2.imshow(self.image)
		#cv2.imwrite("")

	def publish(self):
		img_pub = Image()
		img_pub.data = self.img_cropped
		self.publish_img.publish(img_pub)

def main():
	cam = Camera_view()
	rospy.loginfo("cam infos started")
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		cam.publish()
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
