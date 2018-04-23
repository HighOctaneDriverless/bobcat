#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class Camera_view():
	def __init__(self):
		#init ros_node
        	rospy.init_node('pfwl', anonymous=True)

		rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
		self.image = np.zeros((480,640,3)) 

		#canny parameters
		self.ratio = 3
		self.kernel_size = 3
		self.threshold = 50 # between 0 and 100

	def callback(self,image):
		np_arr = np.fromstring(image.data, np.uint8)
		self.image = np_arr.reshape((480,640,3))
		self.image = self.image[...,::-1]
		self.canny()
		cv2.imshow('Image',self.image)
		cv2.waitKey(1)

	def show(self):
		cv2.imshow(self.image)

	def canny(self):
		img_gray = cv2.cvtColor(self.image,cv2.COLOR_RGB2GRAY)
		img_gray = cv2.blur(img_gray,(3,3))
		edges = cv2.Canny(img_gray,self.threshold,self.threshold*self.ratio,self.kernel_size)
		cv2.imshow("Canny",self.edges)


def main():
	cam = Camera_view()
	rospy.loginfo("cam infos started")
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		#cam.show()
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
