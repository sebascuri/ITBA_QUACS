#!/usr/bin/env python
# -*- coding: utf-8 -*-


import math

class Trajectory(object):
	"""docstring for Trajectory"""
	def __init__(self, t0):
		super(Trajectory, self).__init__()
		self.t0 = t0

	def x(self, t):
		t = t - self.t0 
		return math.cos(2 * math.pi * t  )
	def y(self, t):
		t = t - self.t0 
		return 0.5*math.sin( 4 * math.pi * t )
	def z(self, t):
		t = t - self.t0 
		return 1 - math.cos( 3 * math.pi * t  )
	def yaw(self, t):
		t = t - self.t0 
		return 4.5 * math.pi * t 

	def vx(self, t):
		t = t - self.t0 
		return - 2 * math.pi  * math.sin( 2 * math.pi * t  )
	def vy(self, t):
		t = t - self.t0 
		return 2 * math.pi  * math.cos( 4 * math.pi * t )
	def vz(self, t):
		t = t - self.t0 
		return 3 * math.pi  * math.sin( 3 * math.pi * t )
	def vyaw(self, t):
		t = t - self.t0 
		return 4.5 * math.pi 