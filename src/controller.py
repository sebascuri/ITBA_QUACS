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
#from ardrone_control.cfg import PID_ControllerConfig, TF_ControllerConfig, ZPK_ControllerConfig


import inspect

#from tf import TransformListener
COMMAND_TIME = 0.01

class Controller(Quadrotor, object):
	"""docstring for Controller"""
	def __init__(self, **kwargs):
		super(Controller, self).__init__(**kwargs)
		self.default_init()
		self.srv = Server(ControllerConfig, self.configure_controller )

		#self.controller_state = False

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
	
	def configure_controller(self, config, level):
		controller_type = config.type 
		controller_direction = config.direction

		if config.direction in self.controller.keys():
			changed = True
			if 'PID' in controller_type:
				 
				self.controller[controller_direction] = getattr(Controllers, controller_type)( 
					Kp = config.proportional,
					Kd = config.derivative,
					Ki = config.integral, 
					periodic = (controller_direction == 'yaw')
					)
			elif 'Zero-Pole Gain' in controller_type:
				self.controller[controller_direction] = Controllers.ZPK( 
					[float(a) for a in config.zeros.split(',')],
					[float(a) for a in config.poles.split(',')],
					gain = float( config.gain ), 
					periodic = (controller_direction == 'yaw')
					)
			elif 'Transfer Function' in controller_type:
				self.controller[controller_direction] = Controllers.TF( 
					[float(a) for a in config.numerator.split(',')],
					[float(a) for a in config.denominator.split(',')],
					periodic = (controller_direction == 'yaw')
					)
			else:
				changed = False

			if changed:
				rospy.logwarn("\nController Configuration Changed! \nFor Safety Reasons it will Deactivate, please reactivate manually!")
				rospy.logwarn('\nNew Controller is {0} in direction {1}'.format(config.type, config.direction))
				rospy.logwarn('\n Controller has {0}'.format( str(self.controller[config.direction]) ))
				self.controller_state = False
			


		return config 

	def default_init(self):
		self.controller_state = False
		PID_PARAMS = rospy.get_param( '/controller/PID_Gains' )
		self.controller = dict()
		for key, controller_dict in PID_PARAMS.items():
			self.controller[ key.lower() ] = Controllers.PID( 
				Kp = controller_dict['P'],
				Ki = controller_dict['I'],
				Kd = controller_dict['D'],
				)
		self.controller['yaw'].periodic = True


		"""
		self.controller = dict(
			x = Controllers.PID( 
				Kp =PID_PARAMS['X']['P'],
				Ki =PID_PARAMS['X']['I'],
				Kd =PID_PARAMS['X']['D'],
				),
			y = Controllers.PID( 
				Kp =PID_PARAMS['Y']['P'],
				Ki =PID_PARAMS['Y']['I'],
				Kd =PID_PARAMS['Y']['D'], ),
			z = Controllers.PID( 
				Kp =PID_PARAMS['Z']['P'],
				Ki =PID_PARAMS['Z']['I'],
				Kd =PID_PARAMS['Z']['D'],),
			yaw = Controllers.PID( 
				Kp =PID_PARAMS['Yaw']['P'],
				Ki =PID_PARAMS['Yaw']['I'],
				Kd =PID_PARAMS['Yaw']['D'],
				periodic = True)
			)
		"""
		rospy.logwarn("\nController Configuration Inited by Default to:\nX Controller is type PID with {0} \nY Controller is type PID with {1}\nZ Controller is type PID with {2} \nYaw Controller is type PID with {3} ".format(
			str(self.controller['x']), str(self.controller['y']), str(self.controller['z']), str(self.controller['yaw']) ) )

		self.controller_state = False

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

		for key, siso_controller in self.controller.items():
			if type(siso_controller) == Controllers.TrajectoryPID:
				if key != 'yaw':
					getattr( siso_controller, method_str)( 
						getattr(odometry.pose.pose.position, key), 
						getattr(odometry.twist.twist.linear, key),
						dt 
						) 
				else:
					getattr( siso_controller, method_str)( 
						euler_dict['yaw'],
						odometry.twist.twist.angular.z,
						dt 
						) 


			elif type(siso_controller) == Controllers.PID:
				if key != 'yaw':
					getattr( siso_controller, method_str)(  getattr(odometry.pose.pose.position, key),  dt ) 
				else:
					getattr( siso_controller, method_str)( euler_dict['yaw'], dt ) 
			else:
				if key != 'yaw':
					getattr( siso_controller, method_str)( getattr(odometry.pose.pose.position, key) )
				else:
					getattr( siso_controller, method_str)( euler_dict['yaw'] ) 

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

def main():
	rospy.init_node('ardrone_controller', anonymous = True)
	Controller()
	rospy.spin()

if __name__ == "__main__": main()