#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
import sys
import math
from skimage.transform import resize 
from skimage import img_as_ubyte
from geometry_msgs.msg import Point
from keras import models
from.optimizers import SGD

class Camera_view():
	def __init__(self):

		#CONSTANTS
		img_h = 96
		img_w = 320
		channels = 3		
		n_labels = 3
		path_pic = "/home/martin2/Documents/Data/saved/models/"

		#init ros_node
        	rospy.init_node('roboevent_station', anonymous=True)
		print("init")
		rospy.Subscriber("/bobcat/roboevent_cam",Image,self.callbackRGB)
		self.img_cropped = np.zeros((96,320,3))
		self.publish_img = rospy.Publisher("/bobcat/roboevent_goalpose", Image, queue_size=1)

		#INITIALIZE KERAS etc.
		self.model = segnet(input_shape=(img_h,img_w,channels))
		optimizer = SGD(lr=0.001,momentum=0.9, decay=0.0005, nesterov=False)
		self.model.compile(loss="categorical_crossentropy",optimizer=optimizer,metrics=['accuracy'])
		print('Model compiling: FINISHED')
		
		#LOAD WEIGHTS
		self.model.load_weights(path_pic+'msegnet_var0_02_03_04_08_10_2803_weights_ep3_b20.hdf5')
		print('Model weights loading: FINISHED')
		
	def callbackRGB(self,image):
		np_arr = np.fromstring(image.data, np.uint8)
		print("before reshape",str(np_arr.shape))
		img_reshaped = np_arr.reshape((96,320,3))
		print("after reshape",str(img_reshaped.shape))
		
		#BGR2RGB
		#img_reshaped = img_reshaped[:,:,::-1]

		#resize to (240,320,3)
		#img_resized = resize(img_reshaped,(240,320))

		#crop to (96,320,3)
		#self.img_cropped = img_resized[143:239,0:320,:3]

		self.show(img_reshaped)	

		self.classify(img_reshaped)
		
	def classify(self,image):
		#PREDICT 
		prediction = self.model.predict(image,verbose=0)
		prediction = prediction.reshape((prediction.shape[0],out_h,out_w,n_labels))
		result = img_as_ubyte(prediction[0,;,;,1])
		
		#PRINT PREDICTION
		self.show(prediction[0])
	
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
		#cam.publish()
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
