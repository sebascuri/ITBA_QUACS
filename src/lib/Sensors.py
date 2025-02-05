#!/usr/bin/env python  
# -*- coding: utf-8 -*-

import utm
import numpy.matlib as matlib

from geometry_msgs.msg import Vector3, Vector3Stamped
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import NavSatFix, Imu, Range, Image
from math import cos, sin, atan2, asin, sqrt	

#CAMERA PROCESSING
import cv2
import cv
import numpy as np
from matplotlib import pyplot as plt

from cv_bridge import CvBridge, CvBridgeError
import scipy.signal
from collections import deque

class GPS(object):
	"""docstring for GPS
	initial heading =  0 deg then x is pointing north 
	initial heading = 90 deg then x is pointing west 
	"""
	def __init__(self, **kwargs):
		super(GPS, self).__init__()
		self.latitude = kwargs.get('latitude', 0.0)
		self.longitude = kwargs.get('longitude', 0.0)
		self.altitude = kwargs.get('altitude', 0.0)

		easting, northing, number, letter = utm.from_latlon( self.latitude, self.longitude )

		self.easting = easting
		self.northing = northing

		self.zero = dict( 
			latitude = 0.0,
			longitude = 0.0,
			altitude = 0.0,
			easting = 0.0, 
			northing = 0.0, 
			x = 0.0,
			y = 0.0, 
			z = 0.0,
			yaw = 0.0,)
		#self.set_enu()
		self.calibrated = False
		self.valid = False

		self.Covariance = kwargs.get('Covariance',  matlib.eye(3) )

	def measure(self, fix_data):
		self.latitude = fix_data.latitude
		self.longitude = fix_data.longitude
		self.altitude = fix_data.altitude

		self.valid = bool( fix_data.status.status + 1 )
		
		self.set_enu()


		i = 0
		for var in fix_data.position_covariance:
			self.Covariance.itemset(i, var)
			i += 1
	
	def calibrate( self, yaw = 0.0, x = 0.0, y = 0.0, z = 0.0 ):
		self.zero['latitude'] = self.latitude
		self.zero['longitude'] = self.longitude
		self.zero['altitude'] = self.altitude
		
		easting, northing, number, letter = utm.from_latlon( self.latitude, self.longitude )
		self.zero['easting'] = easting
		self.zero['northing'] = northing

		self.zero['x'] = x
		self.zero['y'] = y
		self.zero['z'] = z
		self.zero['yaw'] = yaw

		self.calibrated = True  

	def set_enu(self):
		easting, northing, number, letter = utm.from_latlon( self.latitude, self.longitude )
		self.easting = easting - self.zero['easting'] 
		self.northing = northing - self.zero['northing'] 
		#print easting, northing 

	def is_valid( self ):
		return self.valid

	def is_calibrated(self):
		return self.calibrated
		
	def get_vector(self):

		x = self.zero['x'] + self.easting * cos(self.zero['yaw']) - self.northing * sin(self.zero['yaw'])
		y = self.zero['y'] + self.easting * sin(self.zero['yaw']) + self.northing * cos(self.zero['yaw'])
		z = self.zero['z'] + self.altitude - self.zero['altitude']
		return [x, y, z]

	def get_pose(self):
		vector = self.get_vector()
		return dict( x = vector[0], y = vector[1], z = vector[2] )

class Magnetometer(object):
	"""docstring for Magnetometer"""
	def __init__(self,*args, **kwargs):
		super(Magnetometer, self).__init__()
		self.properties = dict()
		if len(args)==1:
			arg = args[0]
			if type(arg)==dict:
				for key, value in arg.items():
					setattr(self, key, value)
			elif type(arg) == tuple or type(arg) == list:
				self.x = arg[0]
				self.y = arg[1]
				self.z = arg[2]
			else: #np vector
				self.x = arg.item(0)
				self.y = arg.item(1)
				self.z = arg.item(2)
		elif len(args)==3:
			self.x = args[0]
			self.y = args[1]
			self.z = args[2]

		for key, value in kwargs.items():
			setattr(self, key, value)

		self.H = sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )

		self.Covariance = kwargs.get('Covariance', 0.4 * matlib.eye(3) )
		self.repetitions = 0

	def __str__(self):
		return str([self.x, self.y, self.z])
	
	def measure(self, mag_raw):
		if type(mag_raw) is type( Vector3Stamped() ):
			mag_raw = mag_raw.vector 
		elif type(mag_raw) is type( Navdata() ):
			aux = Vector3()
			aux.x = mag_raw.magX; aux.y = mag_raw.magY; aux.z = mag_raw.magZ
			mag_raw = aux

		for key in ['x','y','z']:
				if getattr(mag_raw, key) == getattr(self, key):
					self.repetitions += 1/3
				else:
					self.repetitions = 0

				setattr( self, key, getattr(mag_raw, key) )

		aux = self.x 
		self.x = -self.y 
		self.y = aux 

		self.H = sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )
		#self.normalize()
		return self
	
	def normalized(self):
		self.H = sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )
		if self.H != 0:
			other = Magnetometer(x = self.x/self.H, y = self.y/self.H, z=self.z/self.H)
		else:
			other = Magnetometer(x = 0., y = 0., z=0.)

		return other

	def get_yaw(self, **kwargs):
		roll = kwargs.get('roll', 0.0)
		pitch = kwargs.get('pitch', 0.0)

		return atan2(self.z * sin(roll) - self.y * cos(roll), 
			self.x * cos(pitch) + (self.y * sin(roll) + self.z * cos(roll) ) * sin(pitch)
			)

	def get_quaternion(self):
		return (self.x, self.y, self.z, 0)
	
	def get_vector(self):
		return [self.x, self.y, self.z]

	def is_working( self ):
		return self.repetitions < 20
	
	@property 
	def x(self):
		return self.properties.get('x', 0.)
	@x.setter 
	def x(self, x):
		self.properties['x'] = x
	@x.deleter
	def x(self):
		del self.properties['x']

	@property 
	def y(self):
		return self.properties.get('y', 0.)
	@y.setter 
	def y(self, y):
		self.properties['y'] = y
	@y.deleter
	def y(self):
		del self.properties['y']

	@property 
	def z(self):
		return self.properties.get('z', 0.)
	@z.setter 
	def z(self, z):
		self.properties['z'] = z
	@z.deleter
	def z(self):
		del self.properties['z']

class Gyroscope(object):
	"""docstring for Gyroscope"""
	def __init__(self,*args, **kwargs):
		super(Gyroscope, self).__init__()
		self.properties = dict()
		if len(args)==1:
			arg = args[0]
			if type(arg)==dict:
				for key, value in arg.items():
					setattr(self, key, value)
			elif type(arg) == tuple or type(arg) == list:
				self.yaw = arg[0]
				self.pitch = arg[1]
				self.roll = arg[2]
			else: #np vector
				self.yaw = arg.item(0)
				self.pitch = arg.item(1)
				self.roll = arg.item(2)
		elif len(args)==3:
			self.yaw = args[0]
			self.pitch = args[1]
			self.roll = args[2]

		for key, value in kwargs.items():
			setattr(self, key, value)


		self.Covariance = kwargs.get('Covariance', 0.4 * matlib.eye(3) )

	def __str__(self):
		return str([self.roll, self.pitch, self.yaw])

	def measure(self, imu_raw):
		self.yaw = imu_raw.angular_velocity.z 
		self.pitch = imu_raw.angular_velocity.y 
		self.roll = imu_raw.angular_velocity.x 

		return self
	
	def get_quaternion(self):
		return (self.roll, self.pitch, self.yaw, 0)
	
	def get_vector(self):
		return [self.roll, self.pitch, self.yaw]
	
	@property 
	def roll(self):
		return self.properties.get('roll', 0.)
	@roll.setter 
	def roll(self, roll):
		self.properties['roll'] = roll
	@roll.deleter
	def roll(self):
		del self.properties['roll']

	@property 
	def pitch(self):
		return self.properties.get('pitch', 0.)
	@pitch.setter 
	def pitch(self, pitch):
		self.properties['pitch'] = pitch
	@pitch.deleter
	def pitch(self):
		del self.properties['pitch']

	@property 
	def yaw(self):
		return self.properties.get('yaw', 0.)
	@yaw.setter 
	def yaw(self, yaw):
		self.properties['yaw'] = yaw
	@yaw.deleter
	def yaw(self):
		del self.properties['yaw']

class Accelerometer(object):
	"""docstring for Accelerometer"""
	def __init__(self, *args, **kwargs):
		super(Accelerometer, self).__init__()
		self.properties = dict()
		if len(args)==1:
			arg = args[0]
			if type(arg)==dict:
				for key, value in arg.items():
					setattr(self, key, value)
			elif type(arg) == tuple or type(arg) == list:
				self.x = arg[0]
				self.y = arg[1]
				self.z = arg[2]
			else: #np vector
				self.x = arg.item(0)
				self.y = arg.item(1)
				self.z = arg.item(2)
		elif len(args)==3:
			self.x = args[0]
			self.y = args[1]
			self.z = args[2]

		for key, value in kwargs.items():
			setattr(self, key, value)

		self.g = sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )

		self.Covariance = kwargs.get('Covariance', 0.4 * matlib.eye(3) )

	def __str__(self):
		return str([self.x, self.y, self.z])

	def measure(self, imu_raw):
		#need to substract g?
		self.x = imu_raw.linear_acceleration.x 
		self.y = imu_raw.linear_acceleration.y
		self.z = imu_raw.linear_acceleration.z  

		self.g = sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )

		return self

	def normalized(self):
		self.g = sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )
		if self.g != 0:
			other = Accelerometer(x = self.x/self.g, y = self.y/self.g, z=self.z/self.g)
		else:
			other = Accelerometer(x = 0., y = 0., z=0.)

		return other
	
	def get_pitch(self):
		return atan2( self.y, sqrt( self.x **2 + self.z ** 2) ) 

	def get_roll(self):
		return atan2( -self.x, self.z ) 

	def get_quaternion(self):
		return (self.x, self.y, self.z, 0)

	def get_vector(self):
		return [self.x, self.y, self.z]
	
	@property 
	def x(self):
		return self.properties.get('x', 0.)
	@x.setter 
	def x(self, x):
		self.properties['x'] = x
	@x.deleter
	def x(self):
		del self.properties['x']

	@property 
	def y(self):
		return self.properties.get('y', 0.)
	@y.setter 
	def y(self, y):
		self.properties['y'] = y
	@y.deleter
	def y(self):
		del self.properties['y']

	@property 
	def z(self):
		return self.properties.get('z', 0.)
	@z.setter 
	def z(self, z):
		self.properties['z'] = z
	@z.deleter
	def z(self):
		del self.properties['z']
		
class Range(object):
	"""docstring for Range"""
	def __init__(self, **kwargs):
		super(Range, self).__init__()
		self.max_range = kwargs.get('max_range', 3000.0)
		self.min_range = kwargs.get('min_range', 20.0)
		self.range = kwargs.get('range', 0.0)
		self.velocity = 0;

		self.min_safe = kwargs.get('min_safe', 0.0)

	def measure(self, range_data, dt):
		if type(range_data) is type( Navdata() ):
			aux =  range_data.altd / 1000.0
		else:
			aux = range_data.range

		self.velocity = (aux - self.range) / dt 
		self.range = aux

	def isFar(self):
		return self.range >= self.max_range 

	def isNear(self):
		return self.range <= self.min_range or self.range <= self.min_safe 	
	
class Camera(object):
	"""docstring for Camera"""
	def __init__(self, *kwargs):
		super(Camera, self).__init__()

	def convert_image( self, ros_image_msg):

		return CvBridge().imgmsg_to_cv2(ros_image_msg, desired_encoding="passthrough")

	def measure(self, ros_image_msg): 
		cv_image = self.convert_image( ros_image_msg )

	def convert_to_gray(self, color_image):
		return cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	
	def fft(self, gray_image):
		f = np.fft.fft2(gray_image)
		return f 
	

def main():
	camera = Camera()
	img = list()
	f = list()

	img.append( cv2.imread('frame0000.jpg', 0) )
	img.append( cv2.imread('frame0001.jpg', 0) )

	# Initiate STAR detector
	orb = cv2.ORB()
	kp = list()
	for image in img:
		f.append( camera.fft( image ) )
		aux = orb.detect( image, None)

		aux, des = orb.compute(image, aux)

		kp.append(des)

	#print cv.FindHomography(  cv.fromarray( kp[0] ),  cv.fromarray( kp[1] ), 0)

	"""
	#Q = scipy.signal.fftconvolve( img[0], img[1] )
	dx, dy = cv2.phaseCorrelate( np.float32(img[0]), np.float32(img[1]) )
	print dx, dy 

	

	# find the keypoints with ORB
	kp = orb.detect(img[0],None)

	# compute the descriptors with ORB
	kp, des = orb.compute(img[0], kp)
	print kp 

	print cv.FindHomography(srcPoints, dstPoints


	# draw only keypoints location,not size and orientation
	#img2 = cv2.drawKeypoints(img[0],kp,color=(0,255,0), flags=0)
	#plt.imshow(img2),plt.show()
	"""

if __name__ == '__main__':
	main()