#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class Camera_view():
	def __init__(self):
		rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
		#self.image = 

	def callback(self,image):
		np_arr = np.fromstring(image.data, np.uint8)
		print(image.encoding)
		#self.image = np.arr.reshape(())


def main():
	cam = Camera_view()
	while not rospy.is_shutdown():
		pass

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass