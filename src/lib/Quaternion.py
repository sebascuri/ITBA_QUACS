#!/usr/bin/env python  
# -*- coding: utf-8 -*-

from math import pi, cos, sin, sqrt, atan2
import numpy as np 
import numpy.matlib as matlib 
import rospy;

def euler_from_quaternion(x, y, z, w):
	r00 = 2.* ( 0.5 - ( y**2 + z**2 ) )
	r10 = 2.* (x * y + z * w)
	r20 = 2.* (x * z - y * w)
	r21 = 2.* (y * z + x * w)
	r22 = 2.* ( 0.5 - ( x**2 + y**2 ) )


	roll = atan2( r21, r22 )
	yaw = atan2( r10, r00 )
	if abs(sin(yaw)) > 0.25:
		pitch = atan2( -r20, r10/sin(yaw) )
	else:
		pitch = atan2( -r20, r00/cos(yaw) )

	return dict(roll = roll, pitch = pitch, yaw = yaw) 

class Quaternion(object):
	"""docstring for Quaternion"""
	def __init__(self, *args, **kwargs):
		super(Quaternion, self).__init__()
		self.properties = dict()
		if len(args) == 1:
			arg = args[0]
			if type(arg) == dict():
				for key, value in arg.items():
					self.properties[key] = value
			elif type(arg) == list or type(arg) == tuple:
				self.x=arg[0]
				self.y=arg[1]
				self.z=arg[2]
				self.w=arg[3]
			else: #np vector
				self.x = arg.item(0)
				self.y = arg.item(1)
				self.z = arg.item(2)
				self.w = arg.item(3)

		for key, value in kwargs.items():
			self.properties[key] = value
		self.norm = self.get_norm()
	def __abs__(self):
		return Quaternion( [abs(self.x), abs(self.y), abs(self.z), abs(self.w)] )
	def __add__(self, other):
		return Quaternion( 
			x = self.x + other.x,
			y = self.y + other.y,
			z = self.z + other.z,
			w = self.w + other.w 
			)
	def __sub__(self, other):
		return Quaternion( 
			x = self.x - other.x,
			y = self.y - other.y,
			z = self.z - other.z,
			w = self.w - other.w 
			)
	def __mul__(self, other):
		if type(other) == type(self):
			new_quaternion =  Quaternion( 
				x =  other.w*self.x + other.z*self.y - other.y*self.z + other.x*self.w,
				y = -other.z*self.x + other.w*self.y + other.x*self.z + other.y*self.w,
				z =  other.y*self.x - other.x*self.y + other.w*self.z + other.z*self.w,
				w = -other.x*self.x - other.y*self.y - other.z*self.z + other.w*self.w
				) 
		else:
			for key, value in self.properties.items():
				setattr(self, key, value * other )
			new_quaternion = self

		return new_quaternion	
	def __eq__(self, other):
		return self.x==other.x and self.y==other.y and self.z==other.z and self.w==other.w 
	def __str__(self):
		return str(self.properties)
	def __iter__(self):
		for key, value in self.properties.items():
			yield key, value

	def conjugate(self):
		other = Quaternion( self.get_quaternion() )
		other.x = -other.x 
		other.y = -other.y 
		other.z = -other.z 

		return other 
	
	def invert(self):
		other = self.conjugate()
		other.normalize()
		return other 

	def rotation_matrix(self):
		return np.mat([ 
			[2.* ( 0.5 - ( self.y**2 + self.z**2 ) ), 2.* (self.x * self.y - self.z * self.w), 2.* (self.x * self.z + self.y * self.w) ],
			[2.* (self.x * self.y + self.z * self.w), 2.* ( 0.5 - ( self.x**2 + self.z**2 ) ), 2.* (self.y * self.z - self.x * self.w) ],
			[2.* (self.x * self.z - self.y * self.w), 2.* (self.y * self.z + self.x * self.w), 2.* ( 0.5 - ( self.x**2 + self.y**2 ) ) ]	
			])
	
	def x_jacobian(self):
		return 2.*np.mat([
			[0, -2*self.y, -2*self.z, 0 ],
			[  self.y,  self.x, self.w, self.z ],
			[  self.z, -self.w, self.x, -self.y],
		 ])

	def y_jacobian(self):
		return 2.*np.mat([
			[ self.y, self.x, -self.w, -self.z],
			[ -2*self.x, 0, -2*self.z, 0 ],
			[ self.w, self.z, self.y, self.x],
		 ])

	def z_jacobian(self):
		return 2.*np.mat([
			[ self.z, self.w, self.x, self.y ],
			[ -self.w, self.z, self.y, -self.x ],
			[ -2*self.x, -2*self.y, 0, 0],
		 ])

	def x_inv_jacobian(self):
		#np.dot(self.invert().x_jacobian(), np.mat([ [-1,0,0, 0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1] ]) )
		return 2.*np.mat([
			[0, -2*self.y, -2*self.z, 0 ],
			[  self.y,  self.x, -self.w, -self.z ],
			[  self.z, self.w, self.x, self.y],
		 ])

	def y_inv_jacobian(self):
		#np.dot(self.invert().y_jacobian(), np.mat([ [-1,0,0, 0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1] ]) )
		return 2.*np.mat([
			[ self.y, self.x, self.w, self.z],
			[ -2*self.x, 0, -2*self.z, 0 ],
			[ -self.w, self.z, self.y, -self.x],
		 ])

	def z_inv_jacobian(self):
		#np.dot(self.invert().z_jacobian(), np.mat([ [-1,0,0, 0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1] ]) )
		return 2.*np.mat([
			[ self.z, -self.w, self.x, -self.y ],
			[ self.w, self.z, self.y, self.x ],
			[ -2*self.x, -2*self.y, 0, 0],
		 ])

	def get_norm(self):
		norm = 0 
		for value in self.properties.values():
			norm += value**2

		norm = sqrt(norm)
		return norm 

	def normalize(self):
		self.norm = self.get_norm()

		if self.norm == 0:
			self.x = 0.0; self.y = 0.0; self.z = 0.0; self.w = 1.0;
		else:
			for key, value in self.properties.items():
				setattr(self, key, value/self.norm )

		return self

	def reversed(self):
		other = Quaternion( self.get_quaternion() )
		other.w = -other.w 
		return other

	def get_quaternion(self):
		return (self.x, self.y, self.z, self.w)
	
	def get_vector(self):
		return [self.x, self.y, self.z]

	def set_quaternion(self, quaternion):
		i = 0
		for key in self.properties.key():
			setattr(self, key, quaternion[i])

	def get_euler(self):
		R = self.rotation_matrix()

		roll = atan2( R[2,1], R[2,2] )
		yaw = atan2( R[1,0], R[0,0] )
		if abs(sin(yaw)) > 0.25:
			pitch = atan2( -R[2,0], R[1,0]/sin(yaw) )
		else:
			pitch = atan2( -R[2,0], R[0,0]/cos(yaw) )

		return dict(roll = roll, pitch = pitch, yaw = yaw) 

	def set_euler(self, euler_dict , order = 'rzyx' ):
		yaw = euler_dict.get('yaw', 0.0)
		pitch = euler_dict.get('pitch', 0.0)
		roll = euler_dict.get('roll', 0.0)

		qz = Quaternion(x =  0.0, y=0.0, z = sin(yaw/2.0), w = cos(yaw/2.0) )
		qy = Quaternion(x =  0.0, y=sin(pitch/2.0), z = 0.0, w = cos(pitch/2.0) )
		qx = Quaternion(x =  sin(roll/2.0), y=0.0, z = 0.0, w = cos(roll/2.0) )

		if order == 'rxyz':
			q = qx * qy * qz
		elif order == 'rxzy':
			q = qx * qz * qy 
		elif order == 'ryxz':
			q = qy * qx * qz 
		elif order == 'ryzx':
			q = qy * qz * qx 
		elif order == 'rzxy':
			q = qz * qx * qy 
		elif order == 'rzyx':
			q = qz * qy * qx 

		for key, value in q:
			setattr(self, key, value)

		return self

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

	@property 
	def w(self):
		return self.properties.get('w', 1.)
	@w.setter 
	def w(self, w):
		self.properties['w'] = w
	@w.deleter
	def w(self):
		del self.properties['w']

def main():
	q1 = Quaternion(x=1., y=-2., z = 3., w = 4.)
	q2 = Quaternion(x=-5., y=6., z = 7., w = 8.)
	q3 = Quaternion([-5,6,7,8])
	print q3
	print abs(q2)

	print q3.x_jacobian() == q3.x_jacobian().T 

	print q3.z_jacobian()

	print (q1*q2) 
	print (q3*q2.invert()).normalize()

    #numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
	
	print q1.set_euler( dict(yaw = 3., pitch=1., roll=2.) , 'ryxz')

	#angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
	#numpy.allclose(angles, [0.123, 0, 0])
	print Quaternion( [0.06146124, 0, 0, 0.99810947] ).get_euler()

	#print q3.normalize().z_jacobian()
	print q3.z_inv_jacobian()
	print -q3.conjugate().z_jacobian()

if __name__ == "__main__": main() 