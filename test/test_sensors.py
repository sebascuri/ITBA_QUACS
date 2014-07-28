#!/usr/bin/env python  
# -*- coding: utf-8 -*-
PKG = 'ardrone_control'
import unittest
import rostest
import sys
#sys.path.append("../src/lib/")

from lib import Sensors
import utm 

from sensor_msgs.msg import Imu, Range, NavSatFix, Image, CameraInfo
from geometry_msgs.msg import Vector3Stamped, Vector3 

import numpy as np 
import math 


## A sample python unit test
class SensorTest(unittest.TestCase):
	## test 1 == 1
	def test_one_equals_one(self):
		self.assertEquals(1, 1, "1!=1")

	def test_gps(self):
		sensor = Sensors.GPS()

	def test_magnetometer(self):
		sensor = Sensors.Magnetometer()
		msg = Vector3Stamped()
		msg.vector.x = 1.0
		msg.vector.y = -2.0
		msg.vector.z = 1.4
		sensor.measure( msg )

		self.assertEquals( sensor.x , msg.vector.x )
		self.assertEquals( sensor.y , msg.vector.y )
		self.assertEquals( sensor.z , msg.vector.z )
		self.assertEquals( sensor.H , np.linalg.norm(sensor.get_vector() ) )

		self.assertAlmostEquals( sensor.get_yaw(), math.atan2(-msg.vector.y, msg.vector.x) )

	def test_accelerometer(self):
		sensor = Sensors.Accelerometer()
		msg = Imu()
		msg.linear_acceleration.x = 0.4
		msg.linear_acceleration.y = 0.9
		msg.linear_acceleration.z = 8.32
		sensor.measure(msg)

		self.assertEquals( sensor.x , msg.linear_acceleration.x )
		self.assertEquals( sensor.y , msg.linear_acceleration.y )
		self.assertEquals( sensor.z , msg.linear_acceleration.z )

		self.assertAlmostEquals( sensor.g, np.linalg.norm(sensor.get_vector() ) )

	def test_gyroscope(self):
		sensor = Sensors.Gyroscope()
		msg = Imu()
		msg.angular_velocity.x = 0.4
		msg.angular_velocity.y = 0.9
		msg.angular_velocity.z = 8.32
		sensor.measure(msg)

		self.assertEquals( sensor.roll , msg.angular_velocity.x )
		self.assertEquals( sensor.pitch , msg.angular_velocity.y )
		self.assertEquals( sensor.yaw , msg.angular_velocity.z )

	def test_range(self):
		sensor = Sensors.Range(min_safe=10)
		msg = Range()
		msg.max_range = 3000
		msg.min_range = 5
		msg.range = 2500

		sensor.measure(msg)

		self.assertEquals(sensor.max_range, msg.max_range)
		self.assertEquals(sensor.min_range, msg.min_range)
		self.assertEquals(sensor.range , msg.range)
		self.assertFalse(sensor.isFar())
		self.assertFalse(sensor.isNear())

		msg.range = 4001
		sensor.measure(msg)
		self.assertTrue(sensor.isFar())
		self.assertFalse(sensor.isNear())

		msg.range = 3
		sensor.measure(msg)
		self.assertFalse(sensor.isFar())
		self.assertTrue(sensor.isNear())

		msg.range = 8
		sensor.measure(msg)
		self.assertFalse(sensor.isFar())
		self.assertTrue(sensor.isNear())

	def test_camera(self):
		sensor = Sensors.Camera()

def main():
	rostest.rosrun(PKG, 'test_quadrotor', SensorTest)

if __name__ == '__main__': main()