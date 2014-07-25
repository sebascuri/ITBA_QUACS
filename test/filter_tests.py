#!/usr/bin/python  
# -*- coding: utf-8 -*-
PKG = 'ardonre_control'
import unittest
import rostest
import sys
sys.path.append("../src/lib/")
from collections import deque
import numpy as np 


from Filter import Digital, Mahoney, Magdwick 


from Sensors import Accelerometer, Gyroscope, Magnetometer
from Quaternion import Quaternion 

from sensor_msgs.msg import Imu 
from geometry_msgs.msg import Vector3Stamped, Vector3 
from math import pi


## A sample python unit test
class FilterTest(unittest.TestCase):
	## test 1 == 1
	def test_one_equals_one(self):
		self.assertEquals(1, 1, "1!=1")

	def test_digital_filter(self):
		import h5py
		with h5py.File('FilterTestData/y_raw.h5', 'r') as f:
			input_list = f['/raw'].value 

		with h5py.File('FilterTestData/y_filter.h5', 'r') as f:
			output_list = f['/filter'].value 

		F = Digital( a = [-2.369513007182038, 2.313988414415879, -1.054665405878567, 0.187379492368185], b = [0.004824343357716, 0.019297373430865, 0.028946060146297, 0.019297373430865, 0.004824343357716])

		i = 0
		for data in input_list:
			F.set_input(data)
			self.assertEquals( data, F.input[-1] )
			if i > 50: 
				self.assertAlmostEquals( output_list[i] , F.get_output(), places=4 )
			i+=1

	def load_data(self):
		sys.path.append("./FilterTestData")
		from read_data import read_file
		self.gyroscope = np.mat(read_file('FilterTestData/Gyroscope.txt'))
		self.accelerometer = np.mat(read_file('FilterTestData/Accelerometer.txt'))
		self.magnetometer = np.mat(read_file('FilterTestData/Magnetometer.txt'))
		self.time = np.mat(read_file('FilterTestData/Time.txt'))

		self.quaternion_madgwick_imu = np.roll( np.mat( read_file('FilterTestData/quaternion_madgwick_imu.txt') ), -1)
	
		self.quaternion_madgwick_marg = np.roll( np.mat( read_file('FilterTestData/quaternion_madgwick_marg.txt') ), -1 )
		self.quaternion_mahoney_imu = np.roll( np.mat( read_file('FilterTestData/quaternion_mahoney_imu.txt') ), -1 )
		self.quaternion_mahoney_marg = np.roll( np.mat( read_file('FilterTestData/quaternion_mahoney_marg.txt') ), -1 )

	def measure_time(self,i):
		return self.time[i+1,0] - self.time[i,0]

	def measure_mag(self, i):
		mag_raw = Vector3Stamped();

		mag_raw.vector.x = self.magnetometer[i, 0]; 
		mag_raw.vector.y = self.magnetometer[i, 1]; 
		mag_raw.vector.z = self.magnetometer[i, 2]

		return mag_raw

	def measure_imu(self, i):
		imu_raw = Imu(); 

		imu_raw.angular_velocity.x = self.gyroscope[i,0] * pi/180 
		imu_raw.angular_velocity.y = self.gyroscope[i,1] * pi/180 
		imu_raw.angular_velocity.z = self.gyroscope[i,2] * pi/180 
		imu_raw.linear_acceleration.x = self.accelerometer[i,0]; 
		imu_raw.linear_acceleration.y = self.accelerometer[i,1]; 
		imu_raw.linear_acceleration.z = self.accelerometer[i,2]
		
		return imu_raw 
		
	def test_Mahoney_imu(self):
		self.load_data()
		F = Mahoney( Kp = 0.5, Ki = 0.);

		quaternion = Quaternion(x=0., y=0., z=0., w=1.0)
		
		sensors = dict( 
			gyroscope = Gyroscope(), 
			accelerometer = Accelerometer(),
		)
		for i in range(100): # range(len(self.time) - 1):
			dt = self.measure_time(i)
			imu_raw = self.measure_imu(i)
			sensors['gyroscope'].measure(imu_raw)
			sensors['accelerometer'].measure(imu_raw)
			quaternion = F.predict(dt, quaternion, sensors['gyroscope'] )
			quaternion = F.correct_accelerometer(dt, quaternion, sensors['accelerometer'])

			self.assertAlmostEquals( quaternion.x, self.quaternion_mahoney_imu[i,0] , places=4 )
			self.assertAlmostEquals( quaternion.y, self.quaternion_mahoney_imu[i,1] , places=4 )
			self.assertAlmostEquals( quaternion.z, self.quaternion_mahoney_imu[i,2] , places=4 )
			self.assertAlmostEquals( quaternion.w, self.quaternion_mahoney_imu[i,3] , places=3 )

	def test_Mahoney_marg(self):
		self.load_data()
		F = Mahoney( Kp = 0.5, Ki = 0.);

		quaternion = Quaternion(x=0., y=0., z=0., w=1.0)
		
		sensors = dict( 
			gyroscope = Gyroscope(), 
			accelerometer = Accelerometer(),
			magnetometer = Magnetometer() 
		)

		for i in range(100): #range(len(self.time) - 1):
			dt = self.measure_time(i)
			mag_raw = self.measure_mag(i)
			imu_raw = self.measure_imu(i)
			sensors['gyroscope'].measure(imu_raw)
			sensors['accelerometer'].measure(imu_raw)
			sensors['magnetometer'].measure(mag_raw)
			quaternion = F.predict(dt, quaternion, sensors['gyroscope'] )
			#quaternion = F.correct(dt, quaternion, sensors['accelerometer'], sensors['magnetometer'] )
			quaternion = F.correct_accelerometer(dt, quaternion, sensors['accelerometer'] )
			quaternion = F.correct_magnetometer(dt, quaternion, sensors['magnetometer'] )


			self.assertAlmostEquals( quaternion.x, self.quaternion_mahoney_marg[i,0] , places=4 )
			self.assertAlmostEquals( quaternion.y, self.quaternion_mahoney_marg[i,1] , places=4 )
			self.assertAlmostEquals( quaternion.z, self.quaternion_mahoney_marg[i,2] , places=4 )
			self.assertAlmostEquals( quaternion.w, self.quaternion_mahoney_marg[i,3] , places=3 )

	def test_Magdwick_imu(self):
		self.load_data()
		F = Magdwick( Beta = 0.1 );

		quaternion = Quaternion(x=0., y=0., z=0., w=1.0)
		
		sensors = dict( 
			gyroscope = Gyroscope(), 
			accelerometer = Accelerometer(),
		)
		for i in range(100): #range(len(self.time) - 1):
			dt = self.measure_time(i)
			imu_raw = self.measure_imu(i)
			sensors['gyroscope'].measure(imu_raw)
			sensors['accelerometer'].measure(imu_raw)
			quaternion = F.predict(dt, quaternion, sensors['gyroscope'] )
			quaternion = F.correct_accelerometer(dt, quaternion, sensors['accelerometer'])

			self.assertAlmostEquals( quaternion.x, self.quaternion_madgwick_imu[i,0] , places=4 )
			self.assertAlmostEquals( quaternion.y, self.quaternion_madgwick_imu[i,1] , places=4 )
			self.assertAlmostEquals( quaternion.z, self.quaternion_madgwick_imu[i,2] , places=4 )
			self.assertAlmostEquals( quaternion.w, self.quaternion_madgwick_imu[i,3] , places=3 )

	def test_Madgwick_marg(self):
		self.load_data()
		F = Magdwick( Beta = 0.1 );

		quaternion = Quaternion(x=0., y=0., z=0., w=1.0)
		
		sensors = dict( 
			gyroscope = Gyroscope(), 
			accelerometer = Accelerometer(),
			magnetometer = Magnetometer() 
		)
		for i in range(100): #range(len(self.time) - 1):
			dt = self.measure_time(i)
			mag_raw = self.measure_mag(i)
			imu_raw = self.measure_imu(i)
			sensors['gyroscope'].measure(imu_raw)
			sensors['accelerometer'].measure(imu_raw)
			sensors['magnetometer'].measure(mag_raw)
			quaternion = F.predict(dt, quaternion, sensors['gyroscope'] )
			#quaternion = F.correct(dt, quaternion, sensors['accelerometer'], sensors['magnetometer'] )
			quaternion = F.correct_accelerometer(dt, quaternion, sensors['accelerometer'])
			quaternion = F.correct_magnetometer(dt, quaternion, sensors['magnetometer'])

			self.assertAlmostEquals( quaternion.x, self.quaternion_madgwick_marg[i,0] , places=1 )
			self.assertAlmostEquals( quaternion.y, self.quaternion_madgwick_marg[i,1] , places=2 )
			self.assertAlmostEquals( quaternion.z, self.quaternion_madgwick_marg[i,2] , places=2 )
			self.assertAlmostEquals( quaternion.w, self.quaternion_madgwick_marg[i,3] , places=3 )

def main():
	rostest.rosrun(PKG, 'test_filters', FilterTest)

if __name__ == '__main__': main()
    