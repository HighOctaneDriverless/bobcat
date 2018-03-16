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

	def callback(self,image):
		np_arr = np.fromstring(image.data, np.uint8)
		self.image = np_arr.reshape((480,640,3))
		self.image = self.image[...,::-1]	
		cv2.imshow('Image',self.image)
		cv2.waitKey(1)

	def show(self):
		cv2.imshow(self.image)


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
