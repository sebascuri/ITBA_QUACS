#!/usr/bin/env python  
# -*- coding: utf-8 -*-

import numpy as np 
from math import sin, cos
from Quaternion import Quaternion 
import rospy;


class ArDroneStates(object):
	STATES = ['Unknown', 'Inited', 'Landed', 'Flying', 'Hovering','Test','Taking off', 'Flying2', 'Landed', 'Looping']
	for name in STATES:
		vars()[name] = STATES.index(name)

	def __init__(self, state = 0):
		self.state = state

	def __eq__(self, other):
		if type(other) == ArDroneStates:
			return self.state == other.state 
		if type(other) == int:
			return self.state == other 
		if type(other) == str:
			return self.STATES[self.state] == other 


class Quadrotor(object):
	"""docstring for Quadrotor"""
	def __init__(self, **kwargs):
		super(Quadrotor, self).__init__()
		self.name = kwargs.get('name', "/local")
		self.position = kwargs.get('position', dict( x=0. , y=0., z=0., yaw=0. ) )
		self.velocity = kwargs.get('velocity', dict( x=0. , y=0., z=0., yaw=0. ) )
		self.orientation = kwargs.get('orientation', Quaternion(x=0., y=0., z=0., w=1.) )
 		self.state = ArDroneStates( kwargs.get('state',ArDroneStates.Unknown ))
 		self.battery = kwargs.get('battery', 100.)
		
	def set_state(self, new_state):

		if type(new_state) == int:
			self.state.state = new_state
		elif type(new_state) == str and new_state in ArDroneStates.STATES:
			setattr(self.state, 'state' , getattr(ArDroneStates, new_state) )
		else:
			self.state.state = ArDroneStates.Unknown

	def get_state( self ):
		return ArDroneStates.STATES[ self.state.state ]

def main():

	
	quad = Quadrotor( position = dict(x=0, y=1, z = 0.5, yaw=0.3,) )
	quad.velocity = dict(x=0, y=1, z = 0.5, yaw=0.1)
	print quad.position


	print quad.get_state()
	quad.set_state(ArDroneStates.Flying)
	print quad.get_state()
	quad.set_state('Hovering')
	print quad.get_state()


	print quad.state == 4
	print quad.state == ArDroneStates.Hovering
	print quad.state == 'Hovering'
	

if __name__ == "__main__": main()