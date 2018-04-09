#!/usr/bin/env python

import rospy
import numpy as np
import math
import copy
from scipy import interpolate
from nav_msgs.msg import OccupancyGrid
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import Pose

#init ros_node
rospy.init_node('occupancy_grid', anonymous=True)

map_msg = OccupancyGrid()
map_msg.header.frame_id = 'base_link'

resolution = 0.1 #mpp
ppm = 10		#ppm
width = 15		#in m
height = 15		#in m
width_p = width * ppm
height_p = height * ppm
width_offset  = width / 2.0
height_offset = height/ 2.0

track_width = 3.5 #is 3 but returns better data with 3.5

def callback_obstackles(data):
	circles = data.circles
	left_poles = []
	right_poles = []
	for i in range(0, len(circles)):
		if(circles[i].true_radius > 0.3 and circles[i].true_radius < 0.5):
			left_poles.append( circles[i])
		elif(circles[i].true_radius < 0.3 and circles[i].true_radius > 0.2):
			right_poles.append(circles[i])
	buid_occupancy_grid(left_poles, right_poles)

def buid_occupancy_grid(left_poles, right_poles):
	grid = np.ndarray((int(width_p), int(height_p)), buffer=np.zeros((int(width_p), int(height_p)), dtype=np.int),
	         dtype=np.int)
	grid.fill(int(-1))
	for i in range(0,int(height_p)):
		for j in range(0, int(width_p)):
				grid[int(j), int(i)] = int(0)

	#add left poles to obstacle_grid map
	for pole in range(len(left_poles)):
		x = left_poles[pole].center.x
		y = left_poles[pole].center.y
		#set_obstacle(grid, x, y, left_poles[pole].true_radius)
	#add right poles to obstacle_grid map
	for pole in range(len(right_poles)):
		x = right_poles[pole].center.x
		y = right_poles[pole].center.y
		#set_obstacle(grid, x, y, right_poles[pole].true_radius)
	#add connections between cones
	poles = sort_poles(left_poles)
	tck = interpolate_spline_left(poles)
	set_spline(grid, tck)

	poles = sort_poles(right_poles)
	tck = interpolate_spline_right(poles)
	set_spline(grid, tck)

	for i in range(width_p*height_p):
		map_msg.data[i] = grid.flat[i]

	map_msg.header.stamp = rospy.Time.now()
	pub_occupancy_grid.publish(map_msg)

def interpolate_spline_left(poles):
		points = [[-1, track_width/2.0]]
		points.append([0, + track_width/2.0])
		points.append([0.2, + track_width/2.0])
		for pole in range(len(poles)):
			points.append([poles[pole].center.x, poles[pole].center.y])
		tmp = np.array(points)
		tmp = np.array([tmp[:,0], tmp[:,1]])
		tck = interpolate.splprep(tmp, s= 0.1)
		return tck

def interpolate_spline_right(poles):
		points = [[-1, - track_width/2.0]]
		points.append([0, - track_width/2.0])
		points.append([0.2, - track_width/2.0])
		for pole in range(len(poles)):
			points.append([poles[pole].center.x, poles[pole].center.y])
		#print(points)
		tmp = np.array(points)
		tmp = np.array([tmp[:,0], tmp[:,1]])
		tck = interpolate.splprep(tmp, s= 0.1)
		return tck

def set_spline(grid, tck):
	ti = np.linspace(0,1,60)
	x,y = interpolate.splev(ti, tck[0], der = 0)
	for i in range(0, 60 ,1):
		set_obstacle(grid,x[i], y[i], 0.1)

def sort_poles(poles_):
	poles = copy.deepcopy(poles_)
	length = len(poles)
	for i in range(length):
		for j in range(length -1, i , -1):
			dist_i = math.sqrt(poles[i].center.x ** 2 + poles[i].center.y ** 2)
			dist_j = math.sqrt(poles[j].center.x ** 2 + poles[j].center.y ** 2)
			if(dist_j < dist_i):
				poles[i], poles[j] = poles[j], poles[i]
	return poles

def set_obstacle(grid, x, y, radius):
	ppm = 1/resolution
	i_, j_ = radius * 2.0 * ppm, radius * 2.0 * ppm
	if(x < - width_offset + radius):
		x = - width_offset + radius
	elif(x > width_offset - radius):
		x = width_offset - radius
	elif(y < - height_offset + radius):
		y = - height_offset + radius
	elif(y > height_offset - radius):
		y = height_offset - radius

	width_ = (x + width_offset)*ppm #- 2* radius)*ppm
	height_ =(y + height_offset)*ppm #- radius)*ppm
	for i in range(int(i_)):
		for j in range(int(j_)):
			grid[int(height_ - radius*ppm + i), int(width_ - radius*ppm + j)] = 100


# init subscriber
rospy.Subscriber("/raw_obstacles", Obstacles, callback_obstackles)
#init publisher
pub_occupancy_grid = rospy.Publisher("/bobcat/occupancy_grid", OccupancyGrid, queue_size=10)


if __name__ == '__main__':
	# set grid parameters
	if rospy.has_param("occupancy_rate"):
		rate = rospy.get_param("occupancy_rate")

	if rospy.has_param("grid_resolution"):
		resolution = rospy.get_param("grid_resolution")

	if rospy.has_param("grid_width"):
		width = rospy.get_param("grid_width")

	if rospy.has_param("grid_height"):
		height = rospy.get_param("grid_height")

	width_p = width * ppm
	height_p = width * ppm
	width_offset = float(width / 2.0)
	height_offset = float(height/2.0)

	print("width_p", width_p)
	print("height_p", height_p)
	print("width", width)
	print("height", height)
	print("width_offset", width_offset)
	print("height_offset", height_offset)
	# fill map_msg with the parameters from launchfile
	map_msg.info.resolution = resolution
	map_msg.info.width = width_p
	map_msg.info.height = height_p
	map_msg.data = range(width_p * height_p)
	map_msg.info.origin = Pose()
	map_msg.info.origin.position.x = - width_offset
	map_msg.info.origin.position.y = - height_offset
	map_msg.info.origin.orientation.w = 1.0

	rospy.loginfo("Occupancy_Grid started")
	rospy.spin()
