#!/usr/bin/env python  
# -*- coding: utf-8 -*-

PKG = 'ardrone_control'
import unittest
import rostest
import sys
sys.path.append("../src/lib/")

from lib.Quadrotor import Quadrotor, ArDroneStates
from lib.Quaternion import Quaternion

## A sample python unit test
class QuadrotorTest(unittest.TestCase):
	## test 1 == 1
	def test_one_equals_one(self):
		self.assertEquals(1, 1, "1!=1")

	def test_state(self):
		quad = Quadrotor()
		i = 0
		for state in ArDroneStates.STATES:
			quad.set_state(state)
			self.assertEquals(quad.state, state)

			quad.set_state(i)
			self.assertEquals(quad.state, state)
			i+=1

	def test_atributes_types(self):
		quad = Quadrotor()
		self.assertEquals(type(quad.name), str)
		self.assertEquals(type(quad.position), dict)
		self.assertEquals(type(quad.velocity), dict)
		self.assertEquals(type(quad.orientation), Quaternion)
		self.assertEquals(type(quad.state), ArDroneStates)
		self.assertEquals(type(quad.battery), float)





def main():
	rostest.rosrun(PKG, 'test_quadrotor', QuadrotorTest)

if __name__ == '__main__': main()