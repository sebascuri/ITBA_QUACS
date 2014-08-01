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
from geometry_msgs.msg import Quaternion as QuaternionMsg

from nav_msgs.msg import Odometry
from ardrone_control.msg import QuadrotorPose

from dynamic_reconfigure.server import Server
from ardrone_control.cfg import InitialConditionsConfig


import tf

import inspect
import subprocess
import math 

COMMAND_TIME = 0.01

class StateEstimation(Quadrotor, object):
	"""docstring for StateEstimation"""
	def __init__(self, **kwargs):
		super(StateEstimation, self).__init__(**kwargs)

		self.sensors = kwargs.get('sensors', dict() )
		self.filters = kwargs.get('filters', dict() )

		self.subscriber = dict(
			raw_navdata = rospy.Subscriber('ardrone/navdata',Navdata, callback = self.recieve_navdata ),
			raw_gps = rospy.Subscriber('fix', NavSatFix, callback = self.recieve_gps),
			raw_imu = rospy.Subscriber('ardrone/imu', Imu, callback = self.recieve_imu ),
			#raw_sonar = rospy.Subscriber('sonar_height', Range, callback = self.recieve_sonar ),
			raw_mag = rospy.Subscriber('ardrone/mag', Vector3Stamped, callback = self.recieve_mag),
			raw_bottom_camera = rospy.Subscriber('ardrone/bottom/image_raw', Image, callback = self.recieve_bottom_camera),
		)

		self.publisher = dict( 
			estimated_pose = rospy.Publisher('ardrone/sensor_fusion/pose', QuadrotorPose),
			estimated_velocity = rospy.Publisher('ardrone/sensor_fusion/velocity', QuadrotorPose),
			estimated_quaternion = rospy.Publisher('ardrone/sensor_fusion/estimated_quaternion', QuaternionMsg),
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
	
	def publish_estimation(self, time):
		#publish estimated pose
		self.publisher['estimated_pose'].publish( self.get_msg(self.position) )
		self.publisher['estimated_velocity'].publish( self.get_msg(self.velocity) ) 
		
		msg = QuaternionMsg()
		for key, value in self.orientation:
			setattr(msg, key, value)

		self.publisher['estimated_quaternion'].publish(msg)

		"""
		msg = Odometry( )
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "/nav"

		msg.child_frame_id = "/local"

		msg.pose.pose.position.x = self.position['x']
		msg.pose.pose.position.y = self.position['y']
		msg.pose.pose.position.z = self.position['z']

		msg.pose.pose.orientation.x = self.orientation.x
		msg.pose.pose.orientation.y = self.orientation.y
		msg.pose.pose.orientation.z = self.orientation.z
		msg.pose.pose.orientation.w = self.orientation.w 

		msg.twist.twist.linear.x = self.velocity['x']
		msg.twist.twist.linear.y = self.velocity['y']
		msg.twist.twist.linear.z = self.velocity['z']

		msg.twist.twist.angular.x = self.velocity['roll']
		msg.twist.twist.angular.y = self.velocity['pitch']
		msg.twist.twist.angular.z = self.velocity['yaw']

		#rospy.loginfo(msgs)

		self.publisher['state'].publish(msg)

		self.tf_broadcaster['local'].sendTransform( (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) , 
			self.orientation.get_quaternion( ), 
			msg.header.stamp,
			msg.child_frame_id, 
			msg.header.frame_id)
		"""
	
	def get_msg( self, attribute ):
		msg = QuadrotorPose()
		for key, value in attribute.items():
			setattr(msg, key, value)

		return msg 

	def update_callback_time( self, callback_name ):
		aux_time = rospy.get_time()
		dt = aux_time - self.callback_time[ callback_name ]
		self.callback_time[ callback_name ] = aux_time
		return dt 

	def recieve_navdata(self, navdata):
		dt = self.update_callback_time( inspect.stack()[0][3] )
		self.set_state(navdata.state) #set state
		# navdata = self.filter_navdata( navdata ) #filter navdata
		#set linear velocity

		if self.state == ArDroneStates.Landed:
			self.velocity['x'] = 0.0
			self.velocity['y'] = 0.0
			self.velocity['z'] = 0.0
		else:
			self.velocity['x'] = navdata.vx / 1000.0 
			self.velocity['y'] = navdata.vy / 1000.0 
			self.velocity['z'] = navdata.vz / 1000.0  #ToDo Correct This

		
		self.battery = navdata.batteryPercent #set battery % 
		
		#predict position update
		self.position = self.filters['position'].predict( dt, self.position, self.velocity)

		#self.position['yaw'] = navdata.rotZ * math.pi / 180.0 

		self.position['z'] = navdata.altd / 1000.0

	def recieve_gps(self, gpsdata):
		pass
		dt = self.update_callback_time( inspect.stack[0][3] )

		self.sensors['gps'].measure( gpsdata )

		self.position = self.filters['position'].correct( self.position, self.sensors['gps'] )
		#transform to local coordinates
		#correct position estimation
		
	def recieve_imu(self, imu_raw):
		dt = self.update_callback_time( inspect.stack()[0][3] )

		self.sensors['gyroscope'].measure( imu_raw )
		self.orientation = self.filters['attitude'].predict( dt, self.orientation, self.sensors['gyroscope'] ) 

		self.sensors['accelerometer'].measure( imu_raw )

		self.orientation = self.filters['attitude'].correct_accelerometer( dt, self.orientation, self.sensors['accelerometer'] ) 
		
		self.position['yaw'] = self.orientation.get_yaw()
		self.velocity['yaw'] = getattr( self.sensors['gyroscope'], 'yaw')

	def recieve_mag(self, mag_raw):
		dt = self.update_callback_time( inspect.stack()[0][3] )

		self.sensors['magnetometer'].measure( mag_raw )
		self.orientation = self.filters['attitude'].correct_magnetometer( dt, self.orientation, self.sensors['magnetometer'] )

	def recieve_sonar(self, range_data):
		""" Receive Sonar Heigh proximity msgs"""
		pass
		dt = self.update_callback_time( inspect.stack()[0][3] )
		self.sensors['range'].measure( range_data )
		
	def recieve_bottom_camera(self, image_raw):
		pass
		dt = self.update_callback_time( inspect.stack()[0][3] )
		
	def filter_navdata( self, navdata ):
		for key, navdata_filter in self.filters['navdata'].items():
			navdata_filter.set_input( getattr(navdata, key) )
			setattr( navdata, key, navdata_filter.get_output() )

		return navdata 

	def calibrate_cameras( self ):

		cmd = ' $(rospack find ardrone_control)/scripts/load_camera_parameters {0}'.format( 
			rospy.get_param('ardrone/version', 2)
			)
		subprocess.call(cmd, shell=True)


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
			sensors['range'] = Sensors.Range()
		if config.GPS:
			self.sensors['gps'] = Sensors.GPS()
		if config.Camera:
			self.sensors['camera'] = Sensors.Camera()

		self.filters = dict( 
			navdata = dict(), 
			attitude = Filter.Magdwick( Beta = rospy.get_param( 'Magdwick', dict( Beta = 0.1) )['Beta'] ), 
			# attitude = Filter.Mahoney( 	Kp = rospy.get_param( 'Mahoney', dict( Kp = 0.5) )['Kp'], 
			# 		 					Ki = rospy.get_param( 'Mahoney', dict( Ki = 0.0.5) )['Ki']
			# 	),
			position = Filter.GPS_Odometry( Q = rospy.get_param( 'GPSOdometry', dict( Q = 0.1) )['Q'], 
											R = rospy.get_param( 'GPSOdometry', dict( R = 0.01) )['R']  
											) 
			)

		navdata_filter_params = rospy.get_param( 'Navdata', dict() )
		for key, values in navdata_filter_params.items():
			self.filters['navdata'][key] = Filter.Digital( a = values['a'], b = values['b'] )

		rospy.logwarn("\nRestarted State Estimation for drone {0} with: \nInitial Postion = {1}\nSensor List: {2}".format(
			self.name, self.position, self.sensors.keys()))
		return config

def main(argv):
	rospy.init_node('state_estimation', anonymous = True)
	StateEstimation()

	rospy.spin()

if __name__ == '__main__': main( sys.argv[1:] )