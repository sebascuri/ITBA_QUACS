#!/usr/bin/env python  
# -*- coding: utf-8 -*-

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
#import roslib; roslib.load_manifest('ardrone_control')
import rospy
import sys

from lib import Quadrotor, ArDroneStates
from lib import Quaternion

from lib import Filter
from lib import Sensors

from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import NavSatFix, Imu, Range, Image
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import QuaternionStamped as QuaternionMsg

from std_srvs.srv import Empty

from nav_msgs.msg import Odometry
from ardrone_control.msg import QuadrotorPose

from dynamic_reconfigure.server import Server
from ardrone_control.cfg import InitialConditionsConfig


import tf

import inspect
import subprocess
import math 
import numpy as np 

IMU_CALIBRATION_TIME = 10
COMMAND_TIME = 0.01
SONAR_ZERO = 0.7

LIST = ['x', 'y', 'z','yaw']

class ImuCalibrator(object):
	"""docstring for ImuCalibrator"""
	def __init__(self, state = False):
		super(ImuCalibrator, self).__init__()
		self.state = state 

		self.gyro = dict( 
			x = np.array( () ), 
			y = np.array( () ), 
			z = np.array( () ), 
			)

		self.accel = dict( 
			x = np.array( () ), 
			y = np.array( () ), 
			z = np.array( () ), 
			)

		self.gyro_bias = np.array( ( rospy.get_param( '/state_estimation/Imu/gyro_bias' ) ) )
		self.accel_bias = np.array( ( rospy.get_param( '/state_estimation/Imu/accel_bias' ) ) )
	
	def get_gyro_bias( self ):
		return list( self.gyro_bias )

	def get_accel_bias( self ):
		return list( self.accel_bias )

	def update( self, imu_msg ):
		for key, array in self.gyro.items():
			self.gyro[key] = np.append( array, getattr(imu_msg.angular_velocity, key) )
		
		for key, array in self.accel.items():
			self.accel[key] = np.append( array, getattr(imu_msg.linear_acceleration, key) )
		
		self.update_bias( )

	def update_bias( self ):

		for key, array in self.gyro.items():
			self.gyro_bias[ LIST.index(key) ] = np.mean( array )

		for key, array in self.accel.items():
			self.accel_bias[ LIST.index(key) ] = np.mean( array )

class StateEstimation(Quadrotor, object):
	"""docstring for StateEstimation"""
	def __init__(self, **kwargs):
		super(StateEstimation, self).__init__(**kwargs)
		self.imu_calibrator = ImuCalibrator()
		self.sensors = kwargs.get('sensors', dict() )
		self.filters = kwargs.get('filters', dict() )

		self.subscriber = dict(
			raw_navdata = rospy.Subscriber('ardrone/navdata',Navdata, callback = self.recieve_navdata ),
			raw_gps = rospy.Subscriber('ardrone/fix', NavSatFix, callback = self.recieve_gps),
			raw_imu = rospy.Subscriber('ardrone/imu', Imu, callback = self.recieve_imu ),
			# raw_sonar = rospy.Subscriber('sonar_height', Range, callback = self.recieve_sonar ),
			# raw_mag = rospy.Subscriber('ardrone/mag', Vector3Stamped, callback = self.recieve_mag),
			# raw_bottom_camera = rospy.Subscriber('ardrone/bottom/image_raw', Image, callback = self.recieve_bottom_camera),
		)

		self.publisher = dict( 
			estimated_pose = rospy.Publisher('ardrone/sensor_fusion/pose', QuadrotorPose),
			estimated_velocity = rospy.Publisher('ardrone/sensor_fusion/velocity', QuadrotorPose),
			estimated_quaternion = rospy.Publisher('ardrone/sensor_fusion/quaternion', QuaternionMsg),
			)

		self.service = dict(
			gps = rospy.Service('ardrone/calibrate_gps', Empty, self.calibrate_gps),
			imu = rospy.Service('ardrone/calibrate_imu', Empty, self.calibrate_imu),
			)

		self.tf_broadcaster = dict( local = tf.TransformBroadcaster( ) )

		self.timer = dict( publish_timer = rospy.Timer(rospy.Duration( COMMAND_TIME ), self.publish_estimation, oneshot=False) )

		now_time = rospy.get_time()
		
		self.callback_time = dict(
			recieve_navdata = now_time, 
			recieve_gps = now_time,
			recieve_imu = now_time,
			recieve_mag = now_time,
			recieve_sonar = now_time,
			recieve_bottom_camera = now_time,
			)

		self.srv = Server(InitialConditionsConfig, self.restart )

		self.calibrate_cameras( )
		self.calibrate_imu( None )
		
		self.calibrate_gps( None )
	
	def publish_estimation(self, time):
		#publish estimated pose and velocity
		self.publisher['estimated_pose'].publish( self.get_msg(self.position) )
		self.publisher['estimated_velocity'].publish( self.get_msg(self.velocity) ) 
		
		#publish quaternion
		msg = QuaternionMsg()
		for key, value in self.orientation:
			setattr(msg.quaternion, key, value)
		msg.header.stamp = rospy.Time.now()
		self.publisher['estimated_quaternion'].publish(msg)

	def get_msg( self, attribute ):
		msg = QuadrotorPose()
		for key, value in attribute.items():
			setattr(msg, key, value)
		msg.header.stamp = rospy.Time.now()
		return msg 

	def update_callback_time( self, callback_name ):
		aux_time = rospy.get_time()
		dt = aux_time - self.callback_time[ callback_name ]
		self.callback_time[ callback_name ] = aux_time
		return dt 

	def recieve_navdata(self, navdata):
		# dt = self.update_callback_time( inspect.stack()[0][3] )
		self.set_state(navdata.state) #set state
		self.battery = navdata.batteryPercent #set battery % 

		self.recieve_sonar( navdata )
		self.recieve_mag( navdata )

		self.velocity['x'] = navdata.vx / 1000.0
		self.velocity['y'] = navdata.vy / 1000.0

		self.position['z'] = navdata.altd / 1000.0

		self.position['yaw'] = navdata.rotZ * math.pi / 180.0
		# self.filters['plane_velocity'].camera_mesure( navdata )
		# self.velocity.update( self.filters['plane_velocity'].get_velocity() )

		# self.filters['vertical_velocity'].range_measure( self.sensors['range'] )
		# self.velocity.update( self.filters['vertical_velocity'].get_velocity() )

	def recieve_gps(self, gpsdata):
		# dt = self.update_callback_time( inspect.stack()[0][3] )

		self.sensors['gps'].measure( gpsdata )

		if self.sensors['gps'].is_calibrated() and self.sensors['gps'].is_valid():
			self.position = self.filters['position'].correct( self.position, self.sensors['gps'] )
		
	def recieve_imu(self, imu_raw): 
		dt = self.update_callback_time( inspect.stack()[0][3] )

		#predict from gyroscope
		self.sensors['gyroscope'].measure( imu_raw )
		# self.orientation = self.filters['attitude'].predict( dt, self.orientation, self.sensors['gyroscope'] ) 

		#correct from accelerometer
		self.sensors['accelerometer'].measure( imu_raw )
		# self.orientation = self.filters['attitude'].correct_accelerometer( dt, self.orientation, self.sensors['accelerometer'] ) 
		
		#get yaw data
		# self.position['yaw'] = self.orientation.get_yaw()
		self.velocity['yaw'] = getattr( self.sensors['gyroscope'], 'yaw')
		
		#predict velocity
		# self.predict_velocity( dt )

		#predict position
		self.predict_position( dt )

		if self.imu_calibrator.state:
			self.imu_calibrator.update( imu_raw )
	
	def recieve_mag(self, mag_raw):
		dt = self.update_callback_time( inspect.stack()[0][3] )

		self.sensors['magnetometer'].measure( mag_raw )
		
		if self.sensors['magnetometer'].is_working():
			self.orientation = self.filters['attitude'].correct_magnetometer( dt, self.orientation, self.sensors['magnetometer'] )
		else:
			rospy.logerr('Magnetometer stopped. Please Reset. {0} repetead measurements'.format(self.sensors['magnetometer'].repetitions))
	
	def recieve_sonar(self, range_data):
		""" Receive Sonar Heigh proximity msgs"""
		dt = self.update_callback_time( inspect.stack()[0][3] )
		self.sensors['range'].measure( range_data, dt )
		
	def recieve_bottom_camera(self, image_raw):
		dt = self.update_callback_time( inspect.stack()[0][3] )
		
	def filter_navdata( self, navdata ):
		for key, navdata_filter in self.filters['navdata'].items():
			navdata_filter.set_input( getattr(navdata, key) )
			setattr( navdata, key, navdata_filter.get_output() )

		return navdata 

	def predict_velocity( self, dt ):
		# Get Unbiased Accelerometer Data
		biased_acceleration = Quaternion( self.sensors['accelerometer'].get_quaternion() )
		bias = Quaternion( self.imu_calibrator.get_accel_bias() )
		unbiased_acceleration = biased_acceleration - bias 
		
		# Project Unbiased Accelerometer Data in Sensor Frame into Local Frame
		local_acceleration = ( self.orientation *  unbiased_acceleration * self.orientation.conjugate() ).get_vector()
		
		# Integrate data
		# self.filters['plane_velocity'].predict( local_acceleration, dt )
		# self.velocity.update( self.filters['plane_velocity'].get_velocity() )

		# self.filters['vertical_velocity'].predict( local_acceleration, dt)
		# self.velocity.update( self.filters['vertical_velocity'].get_velocity() )

	def predict_position( self, dt ):
		#predict position
		self.position = self.filters['position'].predict( dt, self.position, self.velocity)
		
		# self.position['z'] = self.sensors['range'].range

	def calibrate_cameras( self ):

		cmd = ' $(rospack find ardrone_control)/scripts/load_camera_parameters {0}'.format( 
			rospy.get_param('ardrone/version', 2)
			)
		subprocess.call(cmd, shell=True)

	def calibrate_gps( self, empty):
		self.sensors['gps'].calibrate( self.position['yaw'], self.position['x'], self.position['y'], self.position['z'])
		return []

	def calibrate_imu( self, empty):
		self.imu_calibrator.state = True
		rospy.Timer(rospy.Duration(IMU_CALIBRATION_TIME), self.calibrate_imu_end, oneshot=True)
		rospy.logwarn('Starting IMU Calibration. Do Not Move Drone for {0} seconds'.format( IMU_CALIBRATION_TIME))
		return []

	def calibrate_imu_end(self, time):
		self.imu_calibrator.state = False
		rospy.logwarn('Ended IMU Calibration.')
		rospy.logwarn('Gyroscope Bias is {0}'.format(self.imu_calibrator.get_gyro_bias() ) )
		rospy.logwarn('Accelerometer Bias is {0}'.format(self.imu_calibrator.get_accel_bias() ) )

	def restart( self, config, level ):
		self.name = config.name 
		self.position = dict( x = config.x, y = config.y, z = config.z, yaw = config.yaw)
		self.orientation = Quaternion().set_euler( dict(yaw = config.yaw, pitch = 0.0, roll = 0.0) )
		self.velocity = dict( x = 0., y = 0., z = 0., yaw = 0.)
		self.set_state(ArDroneStates.Unknown)

		self.battery = 100

		self.sensors = dict()
		if config.Accelerometer:
			self.sensors['accelerometer'] = Sensors.Accelerometer()
		if config.Gyroscope:
			self.sensors['gyroscope'] = Sensors.Gyroscope()
		if config.Magnetometer:
			self.sensors['magnetometer'] = Sensors.Magnetometer()
		if config.Range:
			self.sensors['range'] = Sensors.Range()
		if config.GPS:
			self.sensors['gps'] = Sensors.GPS()
		if config.Camera:
			self.sensors['camera'] = Sensors.Camera()

		self.filters = dict( 
			navdata = dict(), 
			attitude = Filter.Magdwick( Beta = rospy.get_param( '/state_estimation/Magdwick/Beta' ) ), 
			# attitude = Filter.Mahoney( Kp = rospy.get_param( '/state_estimation/Mahoney/Kp'), 
			# 		 					Ki = rospy.get_param( '/state_estimation/Mahoney/Ki' ) )
			# 	),
			position = Filter.GPS_Odometry( Q = rospy.get_param( '/state_estimation/GPSOdometry/Q'), 
											R = rospy.get_param( '/state_estimation/GPSOdometry/R') ),
			#plane_velocity = Filter.ImuCameraPlaneVelocity(rospy.get_param( '/state_estimation/Velocity/plane')),
			#vertical_velocity = Filter.ImuRangeVerticalVelocity( rospy.get_param( '/state_estimation/Velocity/vertical') ),
			)

		navdata_filter_params = rospy.get_param( '/state_estimation/Navdata', dict() )
		for key, values in navdata_filter_params.items():
			self.filters['navdata'][key] = Filter.Digital( a = values['a'], b = values['b'] )

		rospy.loginfo("\nRestarted State Estimation for drone {0} with: \nInitial Postion = {1}\nSensor List: {2}".format(
			self.name, self.position, self.sensors.keys()))
		return config

def main(argv):
	rospy.init_node('state_estimation', anonymous = True)
	StateEstimation()

	rospy.spin()

if __name__ == '__main__': main( sys.argv[1:] )