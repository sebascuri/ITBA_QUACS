#!/usr/bin/env python  
# -*- coding: utf-8 -*-

from collections import deque
import numpy as np 
from scipy import signal 
import math 
import Filter 

class ControllerState(object):
	STATES = ['Unkown', 'OpenLoop', 'ClosedLoop']
	def __init__(self, *args, **kwargs):
		self.set_state('Unkown')
		if len(args) == 1:
			self.set_state( args[0] )
		if 'state' in kwargs.keys():
			self.set_state( kwargs.get('state') )

	def set_state(self, new_state):
		if type(new_state) == int:
			self.state = ControllerStates.STATES[new_state]
		elif type(new_state) == str and new_state in ControllerStates.STATES:
			self.state = new_state
		else:
			self.state = 'Unknown'

class Digital(Filter.Digital, object):
	"""docstring for Digital"""
	def __init__(self, **kwargs):
		super(Digital, self).__init__( **kwargs )
		self.set_point = kwargs.get('set_point', 0.0)
		self.error = kwargs.get('error', 0.0)
		self.periodic = kwargs.get('periodic', False)

	def input_measurement( self, measurement ):
		self.error = self.set_point - measurement 
		if self.periodic:
			while self.error > math.pi:
				self.error -= 2*math.pi
			while self.error <= -math.pi:
				self.error += 2*math.pi

		return self.set_input( self.error )

	def change_set_point( self, new_set_point ):
		self.set_point = new_set_point

class DigitalTransferFunction(Filter.Digital):
	"""docstring for TransferFunctionDigital"""
	def __init__(self, arg):
		self.den = kwargs.get('a') #list of denominator coefficients
		self.num = kwargs.get('b') #list of numerator coefficients
		super(TransferFunctionDigital, self).__init__(b = num, a = den )
		self.set_point = kwargs.get('set_point', 0.0)
		self.error = kwargs.get('error', 0.0)
		self.periodic = kwargs.get('periodic', False)

	def input_measurement( self, measurement ):
		self.error = self.set_point - measurement 
		if self.periodic:
			while self.error > math.pi:
				self.error -= 2*math.pi
			while self.error <= -math.pi:
				self.error += 2*math.pi

		return self.set_input( self.error )

	def change_set_point( self, new_set_point ):
		self.set_point = new_set_point
	
class ZeroPoleGain(object):
	"""docstring for ZeroPoleGain"""
	def __init__(self, *args, **kwargs):
		super(ZeroPoleGain, self).__init__()

		self.zeros = kwargs.get('zeros')
		self.poles = kwargs.get('zeros')
		self.gain = kwargs.get('zeros')

		self.set_point = kwargs.get('set_point', 0.0)
		self.error = kwargs.get('error', 0.0)
		self.periodic = kwargs.get('periodic', False)



		
class PID(object):
	"""docstring for PID"""
	def __init__(self, **kwargs):
		super(PID, self).__init__()
		self.set_point = dict(
			proportional = kwargs.get('set_point', 0.0), 
			derivative = 0.0
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

		self.measurement = 0.0
	
	def __str__(self):
		return 'Kp = {0}, Kd = {1}, Ki = {2}'.format(self.gains['proportional'], self.gains['derivative'], self.gains['integral'] )
		
	def change_set_point( self, new_set_point, dt = None ):
		if dt is not None:
			self.set_point['derivative'] = ( new_set_point - self.set_point['proportional'] ) / dt
		
		self.set_point['proportional'] = new_set_point	

	def input_measurement( self, measurement, dt ):
		p_error = self.set_point['proportional'] - measurement 
		d_error = self.set_point['derivative'] - (measurement - self.measurement)/dt 
		self.measurement = measurement

		if self.periodic:
			while p_error > math.pi:
				p_error -= 2*math.pi
			while p_error <= -math.pi:
				p_error += 2*math.pi

		self.parallel_errors['proportional'] = p_error 
		self.parallel_errors['derivative'] = d_error
		if not self.saturation:
			self.parallel_errors['integral'] += p_error * dt 

		return self.get_output()

	def get_output(self):
		aux = 0 
		for key in self.parallel_errors.keys():
			aux += self.parallel_errors[key] * self.gains[key]

		return aux 

	def reset(self):
		for key in self.parallel_errors.keys():
			self.parallel_errors[key] = 0.0

class TrajectoryPID(PID, object):
	"""docstring for TrajectoryPID"""
	def __init__(self, **kwargs):
		super(TrajectoryPID, self).__init__()
		self.set_point = dict(
			position = kwargs.get('set_point_position', 0.0), 
			velocity = kwargs.get('set_point_velocity', 0.0),
			)
	
	def change_set_point( self, *args, **kwargs ):
		if len(args) == 1:
			arg = args[0]
			if type(arg) == dict:
				self.change_set_point(**arg)
			elif type(arg) == list or type(arg) == tuple:
				self.change_set_point(arg[0], arg[1])
			elif type(arg) == float:
				self.set_point['position'] = arg
			else: #np vector
				self.change_set_point( arg.item(0), arg.item(1) ) 
		elif len(args) == 2:
			self.set_point['position'] = args[0]
			self.set_point['velocity'] = args[1]


		for key, value in kwargs.items():
			self.set_point[key] = value 

	def input_measurement( self, position_measurement, velocity_measurement, *dt ):
		aux = self.set_point['position'] - position_measurement 

		if self.periodic:
			while aux > math.pi:
				aux -= 2*math.pi
			while aux <= -math.pi:
				aux += 2*math.pi
		self.parallel_errors['proportional'] = aux 
		self.parallel_errors['derivative'] = self.set_point['velocity'] - velocity_measurement
		
		if not self.saturation:
			self.parallel_errors['integral'] += aux * dt[0]

		return self.get_output()


def main():
	c = PID()
	print getattr(c, 'input_measurement')(2, 0.1)
	print "here too"

	
if __name__ == '__main__':
	main()