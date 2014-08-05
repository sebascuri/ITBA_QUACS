#!/usr/bin/env python  
# -*- coding: utf-8 -*-

from collections import deque
import numpy as np 
from scipy import signal 
import math 
import Filter 

class ControllerState(object):
	def __init__(self, *args, **kwargs):
		self.on = False
		self.position = False

	def set_state(self, new_state):
		self.on = new_state.on 
		self.position = new_state.position

class Digital(Filter.Digital, object):
	"""docstring for Digital"""
	def __init__(self, **kwargs):
		super(Digital, self).__init__()
		self.set_point = kwargs.get('set_point', 0.0)
		self.error = kwargs.get('error', 0.0)
		self.periodic = kwargs.get('periodic', False)
	
	def change_set_point_pose( self, new_set_point, dt = None ):
		self.set_point = new_set_point	
		
	def input_pose( self, measurement, dt = None ):
		self.error = self.set_point - measurement 
		if self.periodic:
			self.error = math.atan2( math.sin( self.error ), math.cos( self.error ))

		return self.set_input( self.error )	

class ZPK(Filter.ZPK, object):
	"""docstring for ZPK"""
	def __init__(self, zeros, poles, gain, **kwargs):
		super(ZPK, self).__init__(zeros, poles, gain, **kwargs)
		self.set_point = kwargs.get('set_point', 0.0)
		self.error = kwargs.get('error', 0.0)
		self.periodic = kwargs.get('periodic', False)

		self.zeros = zeros
		self.poles = poles
		self.gain = gain

	def __str__(self):
		return 'zeros {0}, poles {1}, gain {2}'.format(self.zeros, self.poles, self.gain)
	
	def change_set_point_pose( self, new_set_point ):
		self.set_point = new_set_point	

	def input_pose( self, measurement, dt = None ):
		self.error = self.set_point - measurement 
		if self.periodic:
			self.error = math.atan2( math.sin( self.error ), math.cos( self.error ))

		return self.set_input( self.error )

		
class TF(Filter.TF, object):
	"""docstring for TF"""
	def __init__(self, num, den, **kwargs):
		super(TF, self).__init__(num, den, **kwargs)
		self.set_point = kwargs.get('set_point', 0.0)
		self.error = kwargs.get('error', 0.0)
		self.periodic = kwargs.get('periodic', False)
		self.num = num 
		self.den = den 
	def __str__(self):
		return 'numerator = {0}, denominator = {1}'.format( self.num, self.den )

	def change_set_point_pose( self, new_set_point, dt = None ):
		self.set_point = new_set_point	

	def input_pose( self, measurement, dt = None ):
		self.error = self.set_point - measurement 
		if self.periodic:
			self.error = math.atan2( math.sin( self.error ), math.cos( self.error ))

		return self.set_input( self.error )
	
class PID(object):
	"""docstring for PID"""
	def __init__(self, **kwargs):
		super(PID, self).__init__()
		self.set_point = dict(
			position = kwargs.get('set_point_position', 0.0), 
			velocity = kwargs.get('set_point_velocity', 0.0),
			)

		self.periodic = kwargs.get('periodic', False)

		self.parallel_errors = dict(
			proportional = kwargs.get('p_error', 0.), 
			integral = kwargs.get('i_error', 0.), 
			derivative = kwargs.get('d_error', 0.) )

		self.gains = dict(
			proportional = kwargs.get('Kp', 0.), 
			integral = kwargs.get('Ki', 0.), 
			derivative = kwargs.get('Kd', 0.) 
			)

		self.saturation = False
	
	def __str__(self):
		return 'Kp = {0}, Kd = {1}, Ki = {2}'.format(self.gains['proportional'], self.gains['derivative'], self.gains['integral'] )
		
	def change_set_point_pose( self, position_set_point ):
		self.set_point['position'] = position_set_point

	def change_set_point_velocity( self, velocity_set_point ):
		self.set_point['velocity'] = velocity_set_point

	def input_pose( self, position, dt = 0.0):
		self.parallel_errors['proportional'] = self.set_point['position'] - position 

		if self.periodic:
			p_error = math.atan2( 
				math.sin( self.parallel_errors['proportional'] ), 
				math.cos( self.parallel_errors['proportional'] )
				)

		if not self.saturation:
			self.parallel_errors['integral'] += self.parallel_errors['proportional'] * dt 

		return self.get_output()

	def input_velocity( self, velocity ):
		self.parallel_errors['derivative'] = self.set_point['velocity'] - velocity
		return self.get_output()

	def get_output(self):
		aux = 0 
		for key in self.parallel_errors.keys():
			aux += self.parallel_errors[key] * self.gains[key]

		return aux 

	def reset(self):
		for key in self.parallel_errors.keys():
			self.parallel_errors[key] = 0.0

def main():

	c = Digital()
	print c.get_output()
	print getattr(c, 'input_measurement')(2)
	print c.get_output()

	c = ZPK( [1,1],[0.2,0.3],1 )
	print c.get_output()
	print getattr(c, 'input_measurement')(2)
	print c.get_output()

	#print dir(c) #c.state
	c = TF( [1,1],[0.2,0.3] )
	print c.get_output()
	print getattr(c, 'input_measurement')(2)
	print c.get_output()

	#print dir(c) #c.state
	c = PID()
	print getattr(c, 'input_measurement')(2, 0.1)
	print c.get_output()
	print "here too"

	
if __name__ == '__main__':
	main()