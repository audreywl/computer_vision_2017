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
from geometry_msgs.msg import Twist, Vector3, Pose
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

class Follower(object):
    """object for the ROS node that follows"""
    def __init__(self):
        # self.arg = arg
        self.driving = False
        self.waiting = False
        self.analyzing = True
        self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0)) #velocities to publish
        # self.r = rospy.Rate(10) #Execute at 10 Hz
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.fit = FitCurve()
        self.ellipse = Ellipse()
        
        rospy.Subscriber('/bump', Bump, self.process_bump)
        rospy.Subscriber('/odom', Odometry, self.processOdom)
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publish neato velocities
        

    def drive(self):
        """function that executes driving state"""
        pass

    def analyze(self,msg):
        """function that executes analyzing state"""
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # self.fit.find_whiteboard(self.cv_image)
        self.fit.find_points(self.cv_image)

    def wait(self):
        """function that executes waiting state"""
        pass

    def fucking_stop(self):
        """emergency stop function"""
        print 'STOP!'
        self.moves.linear.x = 0.0
        self.moves.angular.z = 0.0
        self.pub.publish(self.moves)

    def run(self):
        rospy.on_shutdown(self.fucking_stop)
        while not rospy.is_shutdown():
            if self.driving:
                self.drive
            elif self.analyzing:
                self.analyze
            else:
                self.wait
            self.r.sleep()

if __name__ == '__main__':
    follower = Follower()
    follower.run()