#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import rospy 

from lib import Quadrotor
from lib import Quaternion
from lib.Quaternion import euler_from_quaternion
from lib import Controllers
from lib import QuadrotorCommands

from nav_msgs.msg import Odometry 
from diagnostic_msgs.msg import KeyValue

from dynamic_reconfigure.server import Server
from ardrone_control.cfg import ControllerConfig

import inspect

#from tf import TransformListener
COMMAND_TIME = 0.01

class Controller(Quadrotor, object):
	"""docstring for Controller"""
	def __init__(self, **kwargs):
		super(Controller, self).__init__(**kwargs)

		self.controller = kwargs.get('controller')

		self.controller_state = False

		self.subscriber = dict(
			estimated_pose = rospy.Subscriber('ardrone/estimated_pose', Odometry, callback = self.recieve_estimation),
			desired_pose = rospy.Subscriber('ardrone/desired_pose', Odometry, callback = self.recieve_desired),
			controller_state = rospy.Subscriber('ardrone/controller_state', KeyValue, callback = self.recieve_state),
		)

		self.commander = QuadrotorCommands.Commands()

		self.timer = dict( 
			publisher = rospy.Timer(rospy.Duration( COMMAND_TIME ), self.publish_twist, oneshot=False) 
			)

		now_time = rospy.get_time()
		
		self.callback_time = dict(
			recieve_estimation = now_time, 
			recieve_desired = now_time
			)

		#self.tfListener = tf.TransformListener()
	"""
	def setup_controller( self, config, level):
		rospy.logwarn("Controller Configuration Changed! \nFor Safety Reasons it will Deactivate, please reactivate manually!")
		self.controller_state = False
		controller_type = config.type
		self.controller = dict( 
			x = getattr(Controllers, controller_type)( 
				Kp = config.proportional_x,
				Kd = config.derivative_x,
				Ki = config.integral_x,
				a = [float(a) for a in (config.num_x).split(',') ],
				b = [float(b) for b in (config.den_x).split(',') ],
				), 
			y = getattr(Controllers, controller_type)( 
				Kp = config.proportional_y,
				Kd = config.derivative_y,
				Ki = config.integral_y,
				a = [float(a) for a in (config.num_y).split(',') ],
				b = [float(b) for b in (config.den_y).split(',') ],
				), 
			z = getattr(Controllers, controller_type)( 
				Kp = config.proportional_z,
				Kd = config.derivative_z,
				Ki = config.integral_z,
				a = [float(a) for a in (config.num_z).split(',') ],
				b = [float(b) for b in (config.den_z).split(',') ],
				), 
			yaw = getattr(Controllers, controller_type)( 
				Kp = config.proportional_yaw,
				Kd = config.derivative_yaw,
				Ki = config.integral_yaw,
				a = [float(a) for a in (config.num_yaw).split(',') ],
				b = [float(b) for b in (config.den_yaw).split(',') ],
				periodic = True
				), 
			)
		return config 
	"""

	def update_callback_time( self, callback_name ):
		aux_time = rospy.get_time()
		dt = aux_time - self.callback_time[ callback_name ]
		self.callback_time[ callback_name ] = aux_time
		return dt 

	def recieve_odometry( self, odometry, method_str , dt = COMMAND_TIME):
		euler_dict = euler_from_quaternion( 
				odometry.pose.pose.orientation.x,
				odometry.pose.pose.orientation.y,
				odometry.pose.pose.orientation.z,
				odometry.pose.pose.orientation.w
				)

		if type(self.controller) == Controllers.TrajectoryPID:
			getattr( self.controller['x'], method_str)( odometry.pose.pose.position.x, odometry.twist.twist.linear.x, dt )
			getattr( self.controller['y'], method_str)( odometry.pose.pose.position.y, odometry.twist.twist.linear.y, dt )
			getattr( self.controller['z'], method_str)( odometry.pose.pose.position.z, odometry.twist.twist.linear.z, dt )
			getattr( self.controller['yaw'], method_str)( euler_dict['yaw'], odometry.twist.twist.angular.z, dt )

		else:
			getattr( self.controller['x'], method_str)( odometry.pose.pose.position.x, dt )
			getattr( self.controller['y'], method_str)( odometry.pose.pose.position.y, dt )
			getattr( self.controller['z'], method_str)( odometry.pose.pose.position.z, dt )
			getattr( self.controller['yaw'], method_str)( euler_dict['yaw'], dt )

	def recieve_estimation( self, odometry ):
		self.recieve_odometry( odometry, 'input_measurement', self.update_callback_time( inspect.stack()[0][3] ) )
		# euler_dict = Quaternion.euler_from_quaternion( 
		# 		odometry.pose.pose.orientation.x,
		# 		odometry.pose.pose.orientation.y,
		# 		odometry.pose.pose.orientation.z,
		# 		odometry.pose.pose.orientation.w
		# 		)

		# if type(self.controller) == Controllers.TrajectoryPID:
		# 	self.controller['x'].input_measurement( odometry.pose.pose.position.x, odometry.twist.twist.linear.x )
		# 	self.controller['y'].input_measurement( odometry.pose.pose.position.y, odometry.twist.twist.linear.y )
		# 	self.controller['z'].input_measurement( odometry.pose.pose.position.z, odometry.twist.twist.linear.z )
		# 	self.controller['yaw'].input_measurement( euler_dict['yaw'], odometry.twist.twist.angular.z )

		# else:
		# 	self.controller['x'].input_measurement( odometry.pose.pose.position.x )
		# 	self.controller['y'].input_measurement( odometry.pose.pose.position.x )
		# 	self.controller['z'].input_measurement( odometry.pose.pose.position.z )
		# 	self.controller['yaw'].input_measurement( euler_dict['yaw'] )

	def recieve_desired( self, odometry ):
		self.recieve_odometry( odometry, 'change_set_point', self.update_callback_time( inspect.stack()[0][3] ) )

		# euler_dict = Quaternion.euler_from_quaternion( 
		# 		odometry.pose.pose.orientation.x,
		# 		odometry.pose.pose.orientation.y,
		# 		odometry.pose.pose.orientation.z,
		# 		odometry.pose.pose.orientation.w
		# 		)

		# if type(self.controller) == Controllers.TrajectoryPID:
		# 	self.controller['x'].change_set_point( odometry.pose.pose.position.x, odometry.twist.twist.linear.x )
		# 	self.controller['y'].change_set_point( odometry.pose.pose.position.y, odometry.twist.twist.linear.y )
		# 	self.controller['z'].change_set_point( odometry.pose.pose.position.z, odometry.twist.twist.linear.z )
		# 	self.controller['yaw'].change_set_point( euler_dict['yaw'], odometry.twist.twist.angular.z )

		# else:
		# 	self.controller['x'].change_set_point( odometry.pose.pose.position.x )
		# 	self.controller['y'].change_set_point( odometry.pose.pose.position.x )
		# 	self.controller['z'].change_set_point( odometry.pose.pose.position.z )
		# 	self.controller['yaw'].change_set_point( euler_dict['yaw'] )

	def recieve_state( self, key_value ):
		self.controller_state = bool( key_value.value )

	def publish_twist( self, time ):
		if self.controller_state:
			velocity = dict()
			for key, siso_controller in self.controller.items():
				velocity[key] = siso_controller.get_output()
			self.commander.velocity( velocity )

class ControllerDynamicReconfigure(object):
	"""docstring for ControllerDynamicReconfigure"""
	def __init__(self):
		super(ControllerDynamicReconfigure, self).__init__()
		self.controller = Controller( controller = dict() )
		self.srv = Server(ControllerConfig, self.restart )

	def restart(self, config, level):
		for timer_object in self.controller.timer.values():
			timer_object.shutdown()
		for subscriber_object in self.controller.subscriber.values():
			subscriber_object.unregister()

		controller_type = config.type
		controller = dict( 
			x = getattr(Controllers, controller_type)( 
				Kp = config.proportional_x,
				Kd = config.derivative_x,
				Ki = config.integral_x,
				a = [float(a) for a in (config.num_x).split(',') ],
				b = [float(b) for b in (config.den_x).split(',') ],
				), 
			y = getattr(Controllers, controller_type)( 
				Kp = config.proportional_y,
				Kd = config.derivative_y,
				Ki = config.integral_y,
				a = [float(a) for a in (config.num_y).split(',') ],
				b = [float(b) for b in (config.den_y).split(',') ],
				), 
			z = getattr(Controllers, controller_type)( 
				Kp = config.proportional_z,
				Kd = config.derivative_z,
				Ki = config.integral_z,
				a = [float(a) for a in (config.num_z).split(',') ],
				b = [float(b) for b in (config.den_z).split(',') ],
				), 
			yaw = getattr(Controllers, controller_type)( 
				Kp = config.proportional_yaw,
				Kd = config.derivative_yaw,
				Ki = config.integral_yaw,
				a = [float(a) for a in (config.num_yaw).split(',') ],
				b = [float(b) for b in (config.den_yaw).split(',') ],
				periodic = True
				), 
			)
		
		
		rospy.logwarn("\nController Configuration Changed! \nFor Safety Reasons it will Deactivate, please reactivate manually!")
		rospy.logwarn('\nNew Controller is {0}'.format(config.type))
		rospy.logwarn('\n X Controller has {0}'.format( str(controller['x']) ))
		rospy.logwarn('\n Y Controller has {0}'.format( str(controller['y']) ))
		rospy.logwarn('\n Z Controller has {0}'.format( str(controller['z']) ))
		rospy.logwarn('\n Yaw Controller has {0}'.format( str(controller['yaw']) ))

		#print es joda
		self.controller = Controller(controller = controller)

		return config 

def main():
	rospy.init_node('ardrone_controller', anonymous = True)
	ControllerDynamicReconfigure()
	rospy.spin()

if __name__ == "__main__": main()