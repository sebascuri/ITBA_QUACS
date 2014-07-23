#!/usr/bin/env python  
# -*- coding: utf-8 -*-

import numpy as np 
from math import sin, cos
from Quaternion import Quaternion 
import rospy;


class ArDroneStates(object):
	STATES = ['Unknown', 'Inited', 'Landed', 'Flying', 'Hovering','Test','Taking off', 'Flying', 'Landed', 'Looping']

class Quadrotor(object):
	"""docstring for Quadrotor"""
	def __init__(self, **kwargs):
		super(Quadrotor, self).__init__()
		self.name = kwargs.get('name', "/local")
		self.position = kwargs.get('position', dict( x=0. , y=0., z=0., yaw=0., pitch=0., roll=0. ) )
		self.velocity = kwargs.get('velocity', dict( x=0. , y=0., z=0., yaw=0., pitch=0., roll=0. ) )
		self.orientation = kwargs.get('orientation', Quaternion(x=0., y=0., z=0., w=1.) )
 		self.state = kwargs.get('state', 'Unknown')
 		self.battery = kwargs.get('battery', 100.)
		
	def set_state(self, new_state):
		if type(new_state) == int:
			self.state = ArDroneStates.STATES[new_state]
		elif type(new_state) == str and new_state in ArDroneStates.STATES:
			self.state = new_state
		else:
			self.state = 'Unknown'


def main():
	quad = Quadrotor( position = dict(x=0, y=1, z = 0.5, yaw=0.3, pitch=0., roll = 0.) )
	quad.velocity = dict(x=0, y=1, z = 0.5, yaw=0.1, pitch=0.2, roll = 0.3)
	print quad.position


	print quad.state
	quad.set_state(3)
	print quad.state
	quad.set_state('Hovering')
	print quad.state

if __name__ == "__main__": main()