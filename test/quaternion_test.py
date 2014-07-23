#!/usr/bin/python  
# -*- coding: utf-8 -*-
PKG = 'ardonre_control'
import unittest
import rostest
import sys
sys.path.append("../src/lib/")

from Quaternion import Quaternion 

class QuaternionTest(unittest.TestCase):
	## test 1 == 1
	def test_one_equals_one(self):
		self.assertEquals(1, 1, "1!=1")

	def test_quaternion_add(self):
		q1 = Quaternion([1,2,3,4])
		q2 = Quaternion( [-3, 5, 6,-3])

		self.assertEquals( q1+q2 , Quaternion([-2,7,9,1]))

	def test_quaternion_sub(self):
		q1 = Quaternion([1,2,3,4])
		q2 = Quaternion( [-3, 5, 6,-3])

		self.assertEquals( q1-q2 , Quaternion([4,-3,-3,7]))

	def test_quaternion_mul(self):
		q1 = Quaternion([1,2,3,4])
		q2 = Quaternion( [-3, 5, 6,-3])

		self.assertEquals( q1 * q2 , Quaternion([-18  ,  -1  ,  26  , -37]))

	def test_conjugate(self):
		q1 = Quaternion([1,2,3,4])
		self.assertEquals( q1.conjugate() , Quaternion([-1,-2,-3,4]) )

	def test_invert(self):
		q1 = Quaternion([1,2,3,4])
		self.assertAlmostEqual( q1.invert() , Quaternion([-1/5.477225575051661,-2/5.477225575051661,-3/5.477225575051661,4/5.477225575051661]) )

	def test_get_euler(self):
		q1 = Quaternion([ 0.073088055816106 ,  0.084100966083119 ,  0.375686807472770, 0.920024231361232])
		euler_dict = q1.normalize().get_euler()
		self.assertAlmostEqual(euler_dict['yaw'], 0.7854)
		self.assertAlmostEqual(euler_dict['pitch'], 0.1)
		self.assertAlmostEqual(euler_dict['roll'], 0.2)

		q1 = Quaternion([ 0.111149437615016 ,  0.007787099393076 ,  -0.384906366390890, 0.916205355099319])
		euler_dict = q1.normalize().get_euler()
		self.assertAlmostEqual(euler_dict['yaw'], -0.7854)
		self.assertAlmostEqual(euler_dict['pitch'], 0.1)
		self.assertAlmostEqual(euler_dict['roll'], 0.2)

		q1 = Quaternion([ 0.111149437615016 ,  -0.007787099393076 ,  0.384906366390890, 0.916205355099319])
		euler_dict = q1.normalize().get_euler()
		self.assertAlmostEqual(euler_dict['yaw'], 0.7854)
		self.assertAlmostEqual(euler_dict['pitch'], -0.1)
		self.assertAlmostEqual(euler_dict['roll'], 0.2)

		q1 = Quaternion([ 0.073088055816106 ,  -0.084100966083119 ,  -0.375686807472770, 0.920024231361232])
		euler_dict = q1.normalize().get_euler()
		self.assertAlmostEqual(euler_dict['yaw'], -0.7854)
		self.assertAlmostEqual(euler_dict['pitch'], -0.1)
		self.assertAlmostEqual(euler_dict['roll'], 0.2)

		q1 = Quaternion([ -0.111149437615016 ,  0.007787099393076 ,  0.384906366390890, 0.916205355099319])
		euler_dict = q1.normalize().get_euler()
		self.assertAlmostEqual(euler_dict['yaw'], 0.7854)
		self.assertAlmostEqual(euler_dict['pitch'], 0.1)
		self.assertAlmostEqual(euler_dict['roll'], -0.2)

		q1 = Quaternion([ -0.073088055816106 ,  -0.084100966083119 ,  0.375686807472770, 0.920024231361232])
		euler_dict = q1.normalize().get_euler()
		self.assertAlmostEqual(euler_dict['yaw'], 0.7854)
		self.assertAlmostEqual(euler_dict['pitch'], -0.1)
		self.assertAlmostEqual(euler_dict['roll'], -0.2)

		q1 = Quaternion([ -0.111149437615016 ,  -0.007787099393076 ,  -0.384906366390890, 0.916205355099319])
		euler_dict = q1.normalize().get_euler()
		self.assertAlmostEqual(euler_dict['yaw'], -0.7854)
		self.assertAlmostEqual(euler_dict['pitch'], -0.1)
		self.assertAlmostEqual(euler_dict['roll'], -0.2)

	def test_set_euler(self):
		q1 = Quaternion()
		q1.set_euler( dict( yaw=0.7854, pitch=0.1, roll= 0.2) ).normalize()

		self.assertAlmostEqual( q1.x, 0.073088055816106 )
		self.assertAlmostEqual( q1.y, 0.084100966083119 )
		self.assertAlmostEqual( q1.z, 0.375686807472770 )
		self.assertAlmostEqual( q1.w, 0.920024231361232 )

		q1.set_euler( dict( yaw=-0.7854, pitch=0.1, roll= 0.2) ).normalize()

		self.assertAlmostEqual( q1.x, 0.111149437615016 )
		self.assertAlmostEqual( q1.y, 0.007787099393076 )
		self.assertAlmostEqual( q1.z, -0.384906366390890 )
		self.assertAlmostEqual( q1.w, 0.916205355099319 )

		q1.set_euler( dict( yaw=0.7854, pitch=-0.1, roll= 0.2) ).normalize()

		self.assertAlmostEqual( q1.x, 0.111149437615016 )
		self.assertAlmostEqual( q1.y, -0.007787099393076 )
		self.assertAlmostEqual( q1.z, 0.384906366390890 )
		self.assertAlmostEqual( q1.w, 0.916205355099319 )

		q1.set_euler( dict( yaw=-0.7854, pitch=-0.1, roll= 0.2) ).normalize()

		self.assertAlmostEqual( q1.x, 0.073088055816106 )
		self.assertAlmostEqual( q1.y, -0.084100966083119 )
		self.assertAlmostEqual( q1.z, -0.375686807472770 )
		self.assertAlmostEqual( q1.w, 0.920024231361232 )


		q1.set_euler( dict( yaw=0.7854, pitch=0.1, roll= -0.2) ).normalize()

		self.assertAlmostEqual( q1.x, -0.111149437615016 )
		self.assertAlmostEqual( q1.y, 0.007787099393076 )
		self.assertAlmostEqual( q1.z, 0.384906366390890 )
		self.assertAlmostEqual( q1.w, 0.916205355099319 )

		q1.set_euler( dict( yaw=-0.7854, pitch=0.1, roll= -0.2) ).normalize()

		self.assertAlmostEqual( q1.x, -0.073088055816106 )
		self.assertAlmostEqual( q1.y, 0.084100966083119 )
		self.assertAlmostEqual( q1.z, -0.375686807472770 )
		self.assertAlmostEqual( q1.w, 0.920024231361232 )

		q1.set_euler( dict( yaw=0.7854, pitch=-0.1, roll= -0.2) ).normalize()

		self.assertAlmostEqual( q1.x, -0.073088055816106 )
		self.assertAlmostEqual( q1.y, -0.084100966083119 )
		self.assertAlmostEqual( q1.z, 0.375686807472770 )
		self.assertAlmostEqual( q1.w, 0.920024231361232 )

		q1.set_euler( dict( yaw=-0.7854, pitch=-0.1, roll= -0.2) ).normalize()

		self.assertAlmostEqual( q1.x, -0.111149437615016 )
		self.assertAlmostEqual( q1.y, -0.007787099393076 )
		self.assertAlmostEqual( q1.z, -0.384906366390890 )
		self.assertAlmostEqual( q1.w, 0.916205355099319 )

def main():

	rostest.rosrun(PKG, 'test_quaternion', QuaternionTest)

if __name__ == '__main__': main()