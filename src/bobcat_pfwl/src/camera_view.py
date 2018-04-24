#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors.kde import KernelDensity

class Camera_view():
	def __init__(self):
		#init ros_node
        	rospy.init_node('pfwl', anonymous=True)

		self.timedelta = 0
		rospy.Subscriber("/camera/rgb/image_raw",Image,self.callbackRGB)
		self.image = np.zeros((480,640,3)) 
		rospy.Subscriber("/camera/depth/image",Image,self.callbackDepth)

		#images
		#self.rgb_slice = np.zeros((40,640,3))
		#self.d_slice = np.zeros((40,640,1))

		#canny parameters
		self.ratio = 3
		self.kernel_size = 3
		self.threshold = 50 # between 0 and 100

	def callbackRGB(self,image):
		np_arr = np.fromstring(image.data, np.uint8)
		#print(str(np_arr.shape))
		self.image = np_arr.reshape((480,640,3))
		self.image = self.image[...,::-1]
		img_sliced = self.image[240:241,:,:]
		rgb_slice = np.repeat(img_sliced,40,axis=0)
		#self.canny()
		#print(str(img_slice.shape))
		#cv2.imshow('Image',rgb_slice)
		#cv2.waitKey(1)

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
			filename = "../pics/image_t"+str(self.timedelta)+".png"
			cv2.imwrite(filename,self.image)
			self.timedelta = self.timedelta + 1
			print("file saved")
		except: 
			print("error",sys.exc_info()[0])

	def canny(self):
		img_gray = cv2.cvtColor(self.image,cv2.COLOR_RGB2GRAY)
		img_gray = cv2.blur(img_gray,(3,3))
		edges = cv2.Canny(img_gray,self.threshold,self.threshold*self.ratio,self.kernel_size)
		cv2.imshow("Canny",edges)

	def kernel(self, X):
		kde = KernelDensity(kernel='gaussian', bandwidth=0.2).fit(X)
		plt.plot(kde)

	def dbscan(self, values):
		#print(values)
		#print(values.T.shape)
		values = values.T
		db = DBSCAN(eps=2, min_samples=2).fit(values)
		core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
		core_samples_mask[db.core_sample_indices_] = True
		labels = db.labels_

		# Number of clusters in labels, ignoring noise if present.
		n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
		
		# Plot result


		# Black removed and is used for noise instead.
		unique_labels = set(labels)
		colors = [plt.cm.Spectral(each)
          	for each in np.linspace(0, 1, len(unique_labels))]
		for k, col in zip(unique_labels, colors):
    			if k == -1:
       			 # Black used for noise.
        			col = [0, 0, 0, 1]
 				class_member_mask = (labels == k)
 				xy = values[class_member_mask & core_samples_mask]
    				plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col), 						markeredgecolor='k', markersize=14)

    				xy = values[class_member_mask & ~core_samples_mask]
				plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
					markeredgecolor='k', markersize=6)

		plt.title('Estimated number of clusters: %d' % n_clusters_)
		#plt.savefig('clusters.png')		
		plt.show()
		


def main():
	cam = Camera_view()
	rospy.loginfo("cam infos started")
	rate = rospy.Rate(0.5)
	while not rospy.is_shutdown():
		#cam.write()
		#cam.show()
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
