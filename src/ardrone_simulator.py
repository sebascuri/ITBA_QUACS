#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import rospy 
from lib import Quadrotor, ArDroneStates
from lib import Filter

from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import NavSatFix, Imu, Range, Image

from random import random 
import math 

COMMAND_TIME = 0.01
g = 9.81

class Simulator(Quadrotor, object):
	"""docstring for Simulator"""
	def __init__(self):
		super(Simulator, self).__init__()
		
		self.subscriber = dict(
			cmd_vel = rospy.Subscriber('/cmd_vel', Twist, callback = self.recieve_cmd_vel),
			take_off = rospy.Subscriber('/ardrone/takeoff', Empty, callback = self.take_off),
			land = rospy.Subscriber('/ardrone/land', Empty, callback = self.land),
		)

		self.publisher = dict( 
			navdata = rospy.Publisher('/ardrone/navdata', Navdata),
			magnetometer = rospy.Publisher('/ardrone/mag', Vector3Stamped),
			imu = rospy.Publisher('/ardrone/imu', Imu,  ),
			)

		self.set_state( ArDroneStates.Landed )

		num = [4.983*10**-5, 4.967*10**-05]
		den = [1, -1.99, 0.99]


		self.tf = dict( 
			x = Filter.Digital( numerator = num, denominator = den ),
			y = Filter.Digital( numerator = num, denominator = den ), 
			z = Filter.Digital( numerator = num, denominator = den ), 
			yaw = Filter.Digital( numerator = num, denominator = den , periodic = True)
			)

		self.altd = 0.0

		self.old_time =  rospy.get_time()

		self.timer = dict( 
			publisher = rospy.Timer(rospy.Duration( COMMAND_TIME ), self.talk, oneshot=False) 
			)

	def recieve_cmd_vel(self, twist):
		
		self.velocity['x'] = twist.linear.x
		self.velocity['y'] = twist.linear.y
		self.velocity['z'] = twist.linear.z
		self.velocity['yaw'] = twist.angular.z 	

	def take_off(self, empty):
		self.set_state( 6 )
		rospy.Timer(rospy.Duration(1), self.taked_off, oneshot=True)

	def taked_off( self, time ):
		self.set_state( ArDroneStates.Flying )
		self.altd = 0.7

	def land(self, empty):
		rospy.Timer(rospy.Duration(1), self.landed, oneshot=True)

	def landed( self, time ):
		self.set_state( ArDroneStates.Landed )
		self.altd = 0.0

	def talk(self, time):
		self.update()
		self.publish_navdata()
		self.publish_imu()
		self.publish_mag()

	def publish_navdata(self):
		msg = Navdata()
		msg.batteryPercent = self.battery; self.battery = self.battery - 0.001;
		msg.state = ArDroneStates.STATES.index( self.get_state() )
		msg.vx = 1000*self.velocity['x'] + 10*random()
		msg.vy = 1000*self.velocity['y'] + 10*random()
		msg.vz = 0; #self.velocity['z'] + random()

		msg.altd = 1000*self.altd# + 0.1*random()

		msg.ax = self.acceleration['x']# + 0.01*random()
		msg.ay = self.acceleration['y']# + 0.01*random()
		msg.az = self.acceleration['z']/g #( self.acceleration['z'] + 0.1*random() ) / g 

		self.publisher['navdata'].publish(msg)

	def publish_imu(self):
		msg = Imu()

		msg.angular_velocity.x =0 #0.1*random()
		msg.angular_velocity.y =0 #0.1*random()
		msg.angular_velocity.z =self.velocity['yaw'] #0.1*random() + self.velocity['yaw']

		msg.linear_acceleration.z = self.acceleration['x'] #+ 0.01*random()
		msg.linear_acceleration.y = self.acceleration['y'] #+ 0.01*random()
		msg.linear_acceleration.z = ( self.acceleration['z'])/g #+ 0.1*random() ) / g 

		self.publisher['imu'].publish(msg)

	def publish_mag( self ):
		msg = Vector3Stamped()
		msg.vector.x = math.cos( self.position['yaw'] ) #+ 0.1*random()
		msg.vector.y = math.sin( self.position['yaw'] ) #+ 0.1*random()
		msg.vector.z = 0. #0.1*random()
		self.publisher['magnetometer'].publish(msg)

	def publish_gps( self ):
		msg = NavSatFix()
		pass

	def update( self ):
		#aux_time = rospy.get_time()
		#dt = self.old_time - aux_time 
		#self.old_time = aux_time

		#self.position['x'] += dt * ( self.velocity['x'] * math.cos(self.position['yaw'] ) - self.velocity['y'] * math.sin(self.position['yaw']) )
		#self.position['y'] += dt * ( self.velocity['x'] * math.sin(self.position['yaw'] ) + self.velocity['y'] * math.cos(self.position['yaw']) ) 
		#self.position['z'] += dt * ( self.velocity['z'] )
		#self.position['yaw'] += dt * ( self.velocity['yaw'] )


		for key, tf in self.tf.items():
			tf.set_input( self.velocity[key] + 0.1 * random() )

			self.position[key] = tf.get_output_index(-1)
			#self.velocity[key] = tf.get_output_index(-2)
			#self.acceleration[key] = tf.get_output_index(-3)

		#self.acceleration['z'] += g 
		#self.position['yaw'] = 0.0
def main():
	rospy.init_node('ardrone_controller', anonymous = True)
	Simulator()
	rospy.spin()

if __name__ == "__main__": main()