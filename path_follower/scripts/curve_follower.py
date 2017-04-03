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
<<<<<<< HEAD
		self.waiting = False
		self.analyzing = True
		self.has_analyzed = False
		self.bumped = True
=======
		# self.waiting = True
		self.analyzing = False
>>>>>>> 436601230b8a59f85f3d6f7f6755c7ff5ce42606

		self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0)) #velocities to publish
		self.r = rospy.Rate(10) #Execute at 10 Hz
		self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
		self.fit = FitCurve()
		
		rospy.Subscriber('/bump', Bump, self.bump)
		rospy.Subscriber('/camera/image_raw', Image, self.analyze)
<<<<<<< HEAD
		rospy.Subscriber('/STAR_pose', PoseStamped, self.drive)
=======
		rospy.Subscriber('/STAR_pose', Pose, self.wait)
		rospy.Subscriber('/STAR_pose', Pose, self.drive)
>>>>>>> 436601230b8a59f85f3d6f7f6755c7ff5ce42606
		
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publish neato velocities
		
	def drive(self,msg):
		"""function that executes driving state"""
<<<<<<< HEAD
		self.driving = True
		self.analyzing = False
		self.waiting = False
		# self.max_min = [4.5,-1]
		if self.has_analyzed:
			self.moves.linear.x,self.moves.angular.z = self.fit.approx_ellipse.get_velocities((rospy.Time.now()-self.start_time).to_sec())
			print self.moves.linear.x, self.moves.angular.z
			self.pub.publish(self.moves)
			self.pos = [msg.pose.position.x,msg.pose.position.y,msg.pose.orientation.z]
			if self.pos[0] < 0 or self.pos[1] < 0 or self.pos[0] > 4 or self.pos[1] > 4:
				self.driving = False
				self.waiting = True
=======
		# self.max_min = [4.5,-1]
		if self.ellipses is not None:
			self.moves.linear.x,self.moves.angular.z = self.fit.approx_ellipse.get_velocities(i)
			self.pub.publish(self.moves)
			self.pos = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.z]
			if self.pos[0] < 0 or self.pos[1] < 0 or self.pos[0] > 4 or self.pos[1] < 4:
				self.driving = False
				# self.waiting = True
>>>>>>> 436601230b8a59f85f3d6f7f6755c7ff5ce42606
				self.fucking_stop()

	def analyze(self,msg):
		"""function that executes analyzing state"""
		self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		self.fit.find_whiteboard(self.cv_image)
		if not self.analyzing:
			return
		try:
			self.ellipses = self.fit.find_ellipse()
			#self.ellipses = self.fit.find_multiple_ellipses()
			
		except TypeError as e:
			print e
		else:
<<<<<<< HEAD
			print 'ELLIPSE!'
			self.start_time = rospy.Time.now()
			self.has_analyzed = True
		 	self.analyzing = False
		 	self.waiting = True
		
=======
			self.analyzing = False
			self.driving = True
>>>>>>> 436601230b8a59f85f3d6f7f6755c7ff5ce42606

	def wait(self,msg):
		"""function that executes waiting state"""
		# pass
		self.time = rospy.get_time()
		self.STAR_time = msg.pose.header.stamp.sec  
		if self.time - self.STAR_time > 3:
			self.analyzing = True

		# if "time passed since last signal" > "1 minute":
					

	def bump(self, msg):
<<<<<<< HEAD
		#self.state_pub.publish(String(self.state))
=======
>>>>>>> 436601230b8a59f85f3d6f7f6755c7ff5ce42606
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
<<<<<<< HEAD
		# if self.waiting == True:
		# 	self.wait()
		sys.exit(0)
=======
		# self.waiting = True
>>>>>>> 436601230b8a59f85f3d6f7f6755c7ff5ce42606

	def run(self):
		rospy.on_shutdown(self.fucking_stop)
		while not rospy.is_shutdown():
			if self.driving:
				#self.drive
				print 'driving!'
			elif self.analyzing:
				#self.analyze
				print 'analyzing!'
			else:
				#self.wait
				print 'waiting!'
			self.fit.update_img()
			#self.fit.plot_curve()
			self.r.sleep()

	# def run(self):
	# 	#rospy.on_shutdown(self.fucking_stop)
	# 	while not rospy.is_shutdown():
	# 		if self.driving:
	# 		if self.state == Follower.DRIVE:
 #                self.state = self.drive()
 #            elif self.state == Follower.ANALYZE:
 #                self.state = self.analyze()
 #            # elif self.state == Follower.STOPPED:
 #            #     self.state = self.fucking_stop()
 #            elif self.state == Follower.WAITING:
 #                self.state = self.wait()
 #            else:
 #                print "invalid state!!!" # note this shouldn't happen
	# 		self.fit.update_img()
	# 		self.r.sleep()

if __name__ == '__main__':
	follower = Follower()
	follower.run()