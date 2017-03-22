from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.distance import cdist

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
	    self.a = V[:,n]

	def ellipse_center(self):
		a = self.a
	    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
	    num = b*b-a*c
	    x0=(c*d-b*f)/num
	    y0=(a*f-b*d)/num
	    return np.array([x0,y0])

	def ellipse_angle_of_rotation(self):
		a = self.a
	    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
	    if b == 0:
	        if a > c:
	            return 0
	        else:
	            return np.pi/2
	    else:
	        if a > c:
	            return np.arctan(2*b/(a-c))/2
	        else:
	            return np.pi/2 + np.arctan(2*b/(a-c))/2

	def ellipse_axis_length(self):
		a = self.a
	    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
	    up = 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)
	    down1=(b*b-a*c)*( (c-a)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
	    down2=(b*b-a*c)*( (a-c)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
	    res1=np.sqrt(up/down1)
	    res2=np.sqrt(up/down2)
	    return np.array([res1, res2])


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
		self.binary_image = cv2.inRange(self.cv_image, self.hsv_lb, self.hsv_ub)
	    # _, self.good_thresh = cv2.threshold(self.cv_image, self.hsv_lb[2], self.hsv_ub[2], cv2.THRESH_BINARY)

	    contours,_ = cv2.findContours(self.binary_image, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE) #we do contours to make sure subsequent points are neighbors -
		#CV_RETR_EXTERNAL because we only want one contour, and CV_CHAIN_APPROX_NONE b/c we don't want to compress by finding extrema, we want all the points


	def find_curve(self,img):
		pass

if __name__ == '__main__':
	pass
