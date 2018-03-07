#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped,Transform, Vector3, Quaternion
from tf2_sensor_msgs import do_transform_cloud
class CameraTf():
	def __init__(self):
		self.newPointCloud= PointCloud2() 
		self.listener = tf.TransformListener()
		self.pub = rospy.Publisher('/zed_depth/transformation', PointCloud2, queue_size = 1)
		rospy.Subscriber('/zed_depth/depth/points', PointCloud2 , self.callback)
		self.transformer = TransformStamped()
		self.transformer.child_frame_id = '/base_link' 
		self.transformer.transform= Transform()

		self.transformer.transform.rotation = Quaternion()
		self.transformer.transform.translation = Vector3()

		quaternion = tf.transformations.quaternion_from_euler(1.5708,3.14,1.5708)	
		self.transformer.transform.rotation.x = quaternion[0]
		self.transformer.transform.rotation.y = quaternion[1]
		self.transformer.transform.rotation.z = quaternion[2]
		self.transformer.transform.rotation.w = quaternion[3]
		
		self.transformer.transform.translation.x = 0
		self.transformer.transform.translation.y = 0
		self.transformer.transform.translation.z = 0
		

	def publisher(self):
		print "publish data" 
		self.pub.publish(self.newPointCloud)
		

	def callback(self, data):
		# transform data
		#publish data 
		self.transformer.header = data.header
		self.newPointCloud = do_transform_cloud(data, self.transformer)
		#self.newPointCloud = self.listener.transformPointCloud( '/base_link', data)
		transform.publisher()
		

if __name__ == '__main__':
	rospy.init_node('cameraTf')
	transform = CameraTf()
	rospy.spin()
	rate = rospy.Rate(10)
	
	#while not rospy.is_shutdown():
		
	#	transform.publisher()
	#	rate.sleep()


#listener
	#tfBuffer = tf2_ros-buffer()
	#listener = tf2_ros_TransformListener(tfBuffer)
	#points = rospy.Publisher('/zed_depth/depth/points', int[], queue_size=1)
	#coordMatrix = rospy.Publisher('/zed_depth/depth/camera_info/R', float64[9], queue_size = 1)
#transfrmation
	#xachse
	#m = [[coordMatrix[0], coordMatrix[1], coordMatrix[2]][coordMatrix[3], coordMatrix[4], coordMatrix[5]][coordMatrix[6], coordMatrix[7], coordMatrix[8]]]
	#r = [[1, 0, 0][0, 0, -1 ][0, 1, 0]]
	#coord.matmul(r,m)
	#coord = r @ m
	#q = tf.transformations.quaternion_about_axis(1.57, (1, 0, 0))
	#coordMatrix.transform.rotation.x
	#rMatrix = tf.transformations.rotatio
	#coordMatrix.publish(coord);
#broadcaster
	#rospy.Subscriber('camera_Pos' , int[], trans)
	#listener= tf.TransformListener()
	#newPointCloud = listener.transformPointCloud('/base_link', '/zed_depth/depth/points')
	
