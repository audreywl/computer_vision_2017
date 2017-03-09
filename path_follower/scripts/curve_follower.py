#!/usr/bin/env python

"""ROS Node for taking an image of a drawn path and executing that path in a known environment."""

import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import image
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from cv_bridge import cv_bridge
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3, Pose
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

class Follower(object):
    """object for the ROS node that follows"""
    def __init__(self):
        self.arg = arg
        self.driving = False
        self.waiting = False
        self.analyzing = True
        rospy.Subscriber('/bump', Bump, self.process_bump)
        rospy.Subscriber('/odom', Odometry, self.processOdom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publish neato velocities
        self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0)) #velocities to publish
        self.r = rospy.Rate(10) #Execute at 10 Hz


    def drive(self):
        """function that executes driving state"""
        pass

    def analyze(self):
        """function that executes analyzing state"""
        pass

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
