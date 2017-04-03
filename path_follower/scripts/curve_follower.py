#!/usr/bin/env python

"""ROS Node for taking an image of a drawn path and executing that path in a known environment."""

import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from fit_curve import FitCurve, Ellipse
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import sys
import matplotlib.pyplot as plt

class Follower(object):
	"""object for the ROS node that follows"""
	# ANALYZE
	# DRIVE
	# BUMPED
	# WAIT

	def __init__(self):
		rospy.init_node('curve_follower')
		# self.arg = arg
		self.driving = False
		self.waiting = False
		self.analyzing = True
		self.has_analyzed = False
		self.bumped = True

		self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0)) #velocities to publish
		self.r = rospy.Rate(10) #Execute at 10 Hz
		self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
		self.fit = FitCurve()
		
		rospy.Subscriber('/bump', Bump, self.bump)
		rospy.Subscriber('/camera/image_raw', Image, self.analyze)
		rospy.Subscriber('/STAR_pose', PoseStamped, self.drive)
		
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publish neato velocities
		
	def drive(self,msg):
		"""function that executes driving state"""
		self.driving = True
		self.analyzing = False
		self.waiting = False
		# self.max_min = [4.5,-1]
		if self.has_analyzed:
			self.moves.linear.x,self.moves.angular.z = self.fit.approx_ellipse.get_velocities((rospy.Time.now()-self.start_time).to_sec())
			print self.moves.linear.x, self.moves.angular.z
			self.pub.publish(self.moves)
			self.pos = [msg.pose.position.x,msg.pose.position.y,msg.pose.orientation.z]
			if self.pos[0] < -0.5 or self.pos[1] < -0.5 or self.pos[0] > 4 or self.pos[1] > 4:
				self.driving = False
				self.waiting = True
				self.fucking_stop()

	def analyze(self,msg):
		"""function that executes analyzing state"""
		self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		self.fit.find_whiteboard(self.cv_image)

		if not self.analyzing:
			return
		try:
			self.ellipses = self.fit.find_ellipse()
			# if not self.fit.binary_image is None:
			# 	plt.cla()
			# 	chunk = 1
			# 	np.save(outfile, self.fit.curve_chunks[chunk])
			# 	approx_ellipse = Ellipse(self.fit.curve_chunks[chunk][:,0], self.fit.curve_chunks[chunk][:,1])
			# 	approx_ellipse.print_properties()
			# 	plt.plot(self.fit.line_points[:,0],self.fit.line_points[:,1])
			# 	plt.plot(self.fit.curve_chunks[chunk][:,0],self.fit.curve_chunks[chunk][:,1],'r')
			# 	plt.pause(.1)
			#self.ellipses = self.fit.find_multiple_ellipses()
			
		except TypeError as e:
			print e
		else:
			print 'ELLIPSE!'
			self.start_time = rospy.Time.now()
			self.has_analyzed = True
		 	self.analyzing = False
		 	self.waiting = True

	def wait(self,msg):
		"""function that executes waiting state"""
		# pass
		self.time = rospy.rospy.Time.now()
		self.STAR_time = msg.header.stamp.sec  
		print self.STAR_time
		if self.time - self.STAR_time > 3:
			self.analyzing = True

		# if "time passed since last signal" > "1 minute":
					

	def bump(self, msg):
		if (msg.leftFront or
			msg.rightFront or
			msg.rightSide or
			msg.leftSide):
			self.fucking_stop()

	def fucking_stop(self):
		"""emergency stop function"""
		print 'STOP!'
		self.moves.linear.x = 0.0
		self.moves.angular.z = 0.0
		self.pub.publish(self.moves)
		# if self.waiting == True:
		# 	self.wait()
		sys.exit(0)


	def run(self):
		rospy.on_shutdown(self.fucking_stop)
		while not rospy.is_shutdown():
			if self.driving:
				#self.drive
				print 'driving!'
			elif self.analyzing:
				#self.analyze
				print 'analyzing!'
			elif self.waiting:
				#self.wait
				print 'waiting!'
				# print self.STAR_time
			else:
				print 'invalid state!'
			self.fit.update_img()
			#self.fit.plot_curve()
			self.r.sleep()


if __name__ == '__main__':
	follower = Follower()
	follower.run()