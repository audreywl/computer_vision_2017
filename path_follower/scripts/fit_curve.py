from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.distance import cdist

""" This is a script takes information from open_cv and does parameterizing 
	math to follow the path """

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
	    
	    contours,_ = cv2.findContours(self.binary_image, 1, 2)

	def find_curve(self,img):

if __name__ == '__main__':