from cv_bridge import CvBridge
import cv2
import numpy as np
from numpy.linalg import eig, inv
from scipy.spatial.distance import cdist
from pylab import *

""" This is a script takes information from open_cv and does parameterizing
	math to follow the path """

class Ellipse(object):
	"""Finds and minimizes error in a set of points to create an arc that best fits them. has functions to return qualities about the ellipse. Credit to: http://nicky.vanforeest.com/misc/fitEllipse/fitEllipse.html"""
	#TODO: rewrite some of this shit so that it works like a class and quits initializing variables over and over
	#TODO: make methods to return stuff that's useful to the parametric driving algorithm like the tangent and normal vector at a given point
	def __init__(self, x, y):
		self.x = x
		self.y = y
		x = x[:,np.newaxis]
		y = y[:,np.newaxis]
		D =  np.hstack((x*x, x*y, y*y, x, y, np.ones_like(x)))
		S = np.dot(D.T,D)
		C = np.zeros([6,6])
		C[0,2] = C[2,0] = 2; C[1,1] = -1
		E, V =  eig(np.dot(inv(S), C))
		n = np.argmax(np.abs(E))
		self.error = V[:,n] #the error is referred to as "a" in the referenced webpage
		self.b,self.c,self.d,self.f,self.g,self.a = self.error[1]/2, self.error[2], self.error[3]/2, self.error[4]/2, self.error[5], self.error[0]
		self.center()
		self.angle_of_rotation()
		self.axis_length()

	def center(self):
	    denom = self.b*self.b-self.a*self.c
	    self.x0=(self.c*self.d-self.b*self.f)/denom
	    self.y0=(self.a*self.f-self.b*self.d)/denom

	def angle_of_rotation(self):
	    if self.b == 0:
	        if self.a > self.c:
	            self.angle=0
	        else:
	            self.angle=np.pi/2
	    else:
	        if self.a > self.c:
	            self.angle=np.arctan(2*self.b/(self.a-self.c))/2
	        else:
	            self.angle=np.pi/2 + np.arctan(2*self.b/(self.a-self.c))/2

	def axis_length(self):
	    num = 2*(self.a*self.f*self.f+self.c*self.d*self.f+self.g*self.b*self.b-2*self.b*self.d*self.f-self.a*self.c*self.g)
	    axis_a_denom=(self.b*self.b-self.a*self.c)*( (self.c-self.a)*np.sqrt(1+4*self.b*self.b/((self.a-self.c)*(self.a-self.c)))-(self.c+self.a))
	    axis_b_denom=(self.b*self.b-self.a*self.c)*( (self.a-self.c)*np.sqrt(1+4*self.b*self.b/((self.a-self.c)*(self.a-self.c)))-(self.c+self.a))
	    self.axis_a=np.sqrt(num/axis_a_denom)
	    self.axis_b=np.sqrt(num/axis_b_denom)


class FitCurve(object):
	def __init__(self):
		""" Initialize the street sign reocgnizer """
		self.cv_image = None                        # the latest image from the camera
		self.binary_image = None
		self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
		self.hsv_lb = np.array([200, 200, 200])           # hsv lower bound
		self.hsv_ub = np.array([255, 255, 255])     # hsv upper bound

        cv2.namedWindow('video_window')
        cv2.namedWindow('threshold_image')

	def find_whiteboard(self,img):
		self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
		self.gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
		self.binary_image = cv2.inRange(self.cv_image, self.hsv_lb, self.hsv_ub)
		if not self.binary_image is None:
		    cv2.imshow('threshold_image',self.binary_image)
		    cv2.waitKey(5)

	def find_points(self,img):
		self.binary_image = cv2.inRange(self.cv_image, self.hsv_lb, self.hsv_ub) # _, self.good_thresh = cv2.threshold(self.cv_image, self.hsv_lb[2], self.hsv_ub[2], cv2.THRESH_BINARY)
		contours,_ = cv2.findContours(self.binary_image, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE) #we do contours to make sure subsequent points are neighbors - #CV_RETR_EXTERNAL because we only want one contour, and CV_CHAIN_APPROX_NONE b/c we don't want to compress by finding extrema, we want all the points


	def find_curve(self,img):
		pass

if __name__ == '__main__':
	arc = 0.8
	R = np.arange(0,arc*np.pi, 0.01)
	x = 1.5*np.cos(R) + 2 + 0.1*np.random.rand(len(R))
	y = np.sin(R) + 1. + 0.1*np.random.rand(len(R))
	approx_ellipse = Ellipse(x,y)



	xx = approx_ellipse.x0 + approx_ellipse.axis_a/2*np.cos(R)*np.cos(approx_ellipse.angle) - approx_ellipse.axis_b/2*np.sin(R)*np.sin(approx_ellipse.angle)
	yy = approx_ellipse.y0 + approx_ellipse.axis_a/2*np.cos(R)*np.sin(approx_ellipse.angle) + approx_ellipse.axis_b/2*np.sin(R)*np.cos(approx_ellipse.angle)



	plot(x,y)
	plot(xx,yy, color = 'red')
	show()
