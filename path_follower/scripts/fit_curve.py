from cv_bridge import CvBridge
import cv2
import numpy as np
import sympy as sp
from numpy.linalg import eig, inv
from scipy.spatial.distance import cdist
from pylab import *
import matplotlib.pyplot as plt
from tempfile import TemporaryFile

outfile = TemporaryFile()
plt.ion()

""" This is a script takes information from open_cv and does parameterizing
math to follow the path """

class Ellipse(object):
	"""Finds and minimizes error in a set of points to create an arc that best fits them. has functions to return qualities about the ellipse.
	Credit to: http://nicky.vanforeest.com/misc/fitEllipse/fitEllipse.html"""
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
		self.parametric()

	def check_validity(self):
		if isinstance(self.x0, complex) or isinstance(self.y0, complex) or isnan(self.axis_a) or isnan(self.axis_b):
			raise TypeError("An ellipse could not be fit to the path")

	def print_properties(self):
		"""For debugging, print the properties of the fitted ellipse after intializing"""
		print 'center:',self.x0, self.y0
		print 'angle:',self.angle
		print  'axis a:', self.axis_a
		print 'axis_b:', self.axis_b

	def center(self):
		"""find the center of the ellipse"""
		denom = self.b*self.b-self.a*self.c
		self.x0=(self.c*self.d-self.b*self.f)/denom
		self.y0=(self.a*self.f-self.b*self.d)/denom

	def angle_of_rotation(self):
		"""find the angle of rotation of the ellipse"""
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
		"""find the length of the major and minor axes of the ellipse"""
		num = 2*(self.a*self.f*self.f+self.c*self.d*self.d+self.g*self.b*self.b-2*self.b*self.d*self.f-self.a*self.c*self.g)
		axis_a_denom=(self.b*self.b-self.a*self.c)*( (self.c-self.a)*np.sqrt(1+4*self.b*self.b/((self.a-self.c)*(self.a-self.c)))-(self.c+self.a))
		axis_b_denom=(self.b*self.b-self.a*self.c)*( (self.a-self.c)*np.sqrt(1+4*self.b*self.b/((self.a-self.c)*(self.a-self.c)))-(self.c+self.a))
		self.axis_a=np.sqrt(num/axis_a_denom)
		self.axis_b=np.sqrt(num/axis_b_denom)

	def parametric(self):
		"""use the fitted ellipse properties to convert the equation to parametric"""
		t = sp.Symbol("t", positive = True) #set up symbolic equations
		x_of_t = self.x0+self.axis_a*sp.cos(t)*sp.cos(self.angle)-self.axis_b*sp.sin(t)*sp.sin(self.angle)
		y_of_t = self.y0+self.axis_a*sp.cos(t)*sp.sin(self.angle)+self.axis_b*sp.sin(t)*sp.cos(self.angle)
		x_tangent = sp.diff(x_of_t, t)
		y_tangent = sp.diff(y_of_t, t)
		self.tangent_magnitude = sp.sqrt(x_tangent*x_tangent+ y_tangent*y_tangent)
		self.x_tangent_hat = x_tangent/self.tangent_magnitude
		self.y_tangent_hat = y_tangent/self.tangent_magnitude
		self.x_curvature = sp.diff(x_of_t/self.tangent_magnitude, t, 1)
		self.y_curvature = sp.diff(y_of_t/self.tangent_magnitude, t, 1)
		#self.N_of_t = sp.Matrix([x_curvature],[y_curvature],[0])
		# self.T_hat_of_t = sp.Matrix([x_tangent_hat],[y_tangent_hat],[0])
		#self.omega_of_t = self.T_hat_of_t.cross(self.N_of_t)

	def get_velocities(self, current_time):
		"""pass in current time, get what the robot velocity should be"""
		t = sp.Symbol("t", positive = True)
		linear_velocity = self.tangent_magnitude.subs(t, current_time)
		N = sp.Matrix([self.x_curvature.subs(t, current_time),self.y_curvature.subs(t, current_time),0.0])
		T_hat = sp.Matrix([self.x_tangent_hat.subs(t, current_time),self.y_tangent_hat.subs(t, current_time),0.0])
		angular_velocity = T_hat.cross(N)
		return (linear_velocity.evalf(), angular_velocity[2].evalf())

class FitCurve(object):
	def __init__(self):
		""" Initialize the street sign reocgnizer """
		self.cv_image = None                        # the latest image from the camera
		self.binary_image = None
		self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

		self.hsv_lb = np.array([0, 0, 0])           # hsv lower bound
		self.hsv_ub = np.array([245, 245, 245])     # hsv upper bound

		cv2.namedWindow('video_window')
		cv2.namedWindow('threshold_image')
		cv2.createTrackbar('H lb', 'threshold_image', 0, 255, self.set_h_lb)
		cv2.createTrackbar('S lb', 'threshold_image', 0, 255, self.set_s_lb)
		cv2.createTrackbar('V lb', 'threshold_image', 0, 255, self.set_v_lb)

		cv2.createTrackbar('H ub', 'threshold_image', 245, 255, self.set_h_ub)
		cv2.createTrackbar('S ub', 'threshold_image', 245, 255, self.set_s_ub)
		cv2.createTrackbar('V ub', 'threshold_image', 245, 255, self.set_v_ub)

	def set_h_lb(self, val):
		""" set hue lower bound """
		self.hsv_lb[0] = val

	def set_s_lb(self, val):
		""" set saturation lower bound """
		self.hsv_lb[1] = val

	def set_v_lb(self, val):
		""" set value lower bound """
		self.hsv_lb[2] = val

	def set_h_ub(self, val):
		""" set hue upper bound """
		self.hsv_ub[0] = val

	def set_s_ub(self, val):
		""" set saturation upper bound """
		self.hsv_ub[1] = val

	def set_v_ub(self, val):
		""" set value upper bound """
		self.hsv_ub[2] = val

	
	def find_whiteboard(self,img):
		self.cv_image = img
		self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
		self.gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
		self.binary_image = cv2.inRange(self.cv_image, self.hsv_lb, self.hsv_ub)
		if not self.binary_image is None:
			self.contours,_ = cv2.findContours(self.binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #we do contours to make sure subsequent points are neighbors -
			# CV_RETR_EXTERNAL because we only want one contour, and CV_CHAIN_APPROX_NONE b/c we don't want to compress by finding extrema, we want all the points

			self.longest_contour_index, self.longest_contour = max(enumerate(self.contours), key=(lambda x: len(x[1])))

			cv2.drawContours(self.cv_image, self.contours, self.longest_contour_index, (0,255,0), 1)
			

			self.longest_contour = np.squeeze(self.longest_contour)

			leftmost_point_index,leftmost_point = min(enumerate(self.longest_contour), key=(lambda x: x[1][0]))

			double_countour = np.append(self.longest_contour,self.longest_contour, axis=0)

			self.line_points = double_countour[leftmost_point_index:leftmost_point_index+(len(self.longest_contour)/2)]

			cv2.drawContours(self.cv_image, self.contours[self.longest_contour_index][leftmost_point_index:leftmost_point_index+(len(self.longest_contour)/2)], -1, (0,0,255), 1)



	def find_ellipse(self):
		xran = max(self.line_points[:,0]) - min(self.line_points[:,0])
		yran = max(self.line_points[:,1]) - min(self.line_points[:,1])
		xratio = xran/2
		yratio = yran/2

		self.approx_ellipse = Ellipse((self.line_points[:,0]-min(self.line_points[:,0]))/xratio,(self.line_points[:,1]-min(self.line_points[:,1]))/yratio)
		self.approx_ellipse.check_validity()


	def find_multiple_ellipses(self):
		chunk_number = 4
		chunk_size = len(self.line_points)/chunk_number
		self.curve_chunks = []
		self.ellipses = []
		for i in range(0,len(self.line_points), chunk_size):
			try:
				current_chunk = self.line_points[i:i+chunk_size]
				self.curve_chunks.append(current_chunk)
			except IndexError as e:
				current_chunk = self.line_points[i:self.line_points[-1]]
				self.curve_chunks.append(current_chunk)
			# current_ellipse = Ellipse(current_chunk[:,0], current_chunk[:,1])
			# current_ellipse.print_properties()
			# current_ellipse.check_validity()
			# self.ellipses.append(current_ellipse)

	def update_img(self):
		if not self.binary_image is None:
			cv2.imshow('threshold_image',self.binary_image)
			cv2.imshow('video_window',self.cv_image)
			cv2.waitKey(5)
			

	def plot_curve(self):
		if not self.binary_image is None:
			plt.cla()
			chunk = 1
			np.save(outfile, self.curve_chunks[chunk])
			approx_ellipse = Ellipse(self.curve_chunks[chunk][:,0], self.curve_chunks[chunk][:,1])
			approx_ellipse.print_properties()
			plt.plot(self.line_points[:,0],self.line_points[:,1])
			plt.plot(self.curve_chunks[chunk][:,0],self.curve_chunks[chunk][:,1],'r')
			plt.pause(.1)


if __name__ == '__main__':

	#node = FitCurve()
	#img = cv2.imread(img,cv2.IMREAD_GRAYSCALE)
	# node.find_whiteboard(img)
	#print node.find_points(img)
	x = np.array([305, 306, 306, 307, 308, 308, 309, 309, 310, 310, 311, 311, 312, 313, 313, 314, 314, 315, 316, 316, 317, 317, 318, 319, 319, 320, 320, 321, 322, 322, 323, 324, 324, 325, 326, 326, 327, 328, 328],dtype=float)
	y = np.array([221, 220, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 209, 208, 207, 206, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193, 192, 191, 190, 189, 188, 187, 186, 185, 184, 183], dtype=float)
	x = x/100.0
	y = y/100.0
	# for i in range(0,len(x)):
	# 	print x[i]
	# 	x[i] = float(x[i])
	# 	print type(x[i])
	# 	y[i] = float(y[i])/100.0
	# print x
	# print y
	#print type(x)

	arc = 0.8
	R = np.arange(0,arc*np.pi, 0.01)
	x2 = 1.5*np.cos(R) + 2 + 0.1*np.random.rand(len(R))
	y2 = np.sin(R) + 1. + 0.1*np.random.rand(len(R))
	print x2
	print y2
	approx_ellipse = Ellipse(x2,y2)
	print approx_ellipse.get_velocities(3.000045)
	xx = approx_ellipse.x0 + approx_ellipse.axis_a/2*np.cos(R)*np.cos(approx_ellipse.angle) - approx_ellipse.axis_b/2*np.sin(R)*np.sin(approx_ellipse.angle)
	yy = approx_ellipse.y0 + approx_ellipse.axis_a/2*np.cos(R)*np.sin(approx_ellipse.angle) + approx_ellipse.axis_b/2*np.sin(R)*np.cos(approx_ellipse.angle)
	# xx = approx_ellipse.x0 + approx_ellipse.axis_a/2*np.cos(R)*np.cos(approx_ellipse.angle) - approx_ellipse.axis_b/2*np.sin(R)*np.sin(approx_ellipse.angle)
	# yy = approx_ellipse.y0 + approx_ellipse.axis_a/2*np.cos(R)*np.sin(approx_ellipse.angle) + approx_ellipse.axis_b/2*np.sin(R)*np.cos(approx_ellipse.angle)
	plot(x,y,'.')
	#plot(xx,yy, color = 'red')
	show()