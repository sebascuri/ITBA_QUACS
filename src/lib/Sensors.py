#!/usr/bin/env python  
# -*- coding: utf-8 -*-

import utm
import numpy.matlib as matlib

from geometry_msgs.msg import Vector3, Vector3Stamped
from math import cos, sin, atan2, asin, sqrt	


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

		self.zero = dict()
		#self.set_enu()
		self.calibrated_zero = False
		self.calibrated_heading = False

	def measure(self, fix_data):
		self.latitude = fix_data.latitude
		self.longitude = fix_data.longitude
		self.altitude = fix_data.altitude

		if not self.calibrated_zero:
			self.calibrated_zero = True 
			self.zero['latitude'] = self.latitude
			self.zero['longitude'] = self.longitude
			self.zero['altitude'] = self.altitude
			easting, northing, number, letter = utm.from_latlon( self.latitude, self.longitude )
			self.zero['easting'] = easting
			self.zero['northing'] = northing


		self.set_enu()
		
	def set_initial_heading(self, yaw):
		self.zero['heading'] = yaw;
		self.calibrated_heading = True

	def set_enu(self):
		easting, northing, number, letter = utm.from_latlon( self.latitude, self.longitude )
		self.easting = easting - self.zero['easting'] 
		self.northing = northing - self.zero['northing'] 

	def get_pose(self):
		try:
			heading = self.zero['heading']
		except KeyError:
			heading =  0.0

		x = self.easting * cos(heading) - self.northing * sin(heading)
		y = self.easting * sin(heading) + self.northing * cos(heading)
		z = self.altitude
		return dict(x= x, y=y, z=z )

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

	def __str__(self):
		return str([self.x, self.y, self.z])
	def measure(self, mag_raw):
		if type(mag_raw) is type( Vector3Stamped() ):
			self.x = mag_raw.vector.x 
			self.y = mag_raw.vector.y
			self.z = mag_raw.vector.z 
		elif type(mag_raw) is type( Vector3() ): 
			self.x = mag_raw.x 
			self.y = mag_raw.y
			self.z = mag_raw.z
		else:
			self.x = navdata.magX
			self.y = navdata.magY
			self.z = navdata.magZ

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
		self.min_range = kwargs.get('min_range', 0.0)
		self.range = kwargs.get('range', 0.0)

		self.min_safe = kwargs.get('min_safe', 0.0)

	def measure(self, range_data):
		self.max_range = range_data.max_range
		self.min_range = range_data.min_range 

		self.range = range_data.range 

	def isFar(self):
		return self.range >= self.max_range 

	def isNear(self):
		return self.range <= self.min_range or self.range <= self.min_safe 	
	
class Camera(object):
	"""docstring for Camera"""
	def __init__(self, *kwargs):
		super(Camera, self).__init__()
		