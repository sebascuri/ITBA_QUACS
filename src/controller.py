#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import rospy 

from lib import Quadrotor
from lib import Quaternion
from lib.Quaternion import euler_from_quaternion
from lib import Controllers
from lib import QuadrotorCommands

from nav_msgs.msg import Odometry 
#from std_msgs.msg import Bool 

from ardrone_control.msg import QuadrotorPose
from ardrone_control.msg import ControllerState as ControllerStateMsg

from dynamic_reconfigure.server import Server
from ardrone_control.cfg import ControllerConfig
#from ardrone_control.cfg import PID_ControllerConfig, TF_ControllerConfig, ZPK_ControllerConfig


import inspect
import math 
#from tf import TransformListener
COMMAND_TIME = 0.01

class Controller(Quadrotor, object):
	"""docstring for Controller"""
	def __init__(self, **kwargs):
		super(Controller, self).__init__(**kwargs)
		self.default_init()
		self.srv = Server(ControllerConfig, self.configure_controller )

		self.controller_state = Controllers.ControllerState()

		self.subscriber = dict(
			estimated_pose = rospy.Subscriber('ardrone/sensor_fusion/pose', QuadrotorPose, callback = self.recieve_estimated_pose),
			estimated_velocity = rospy.Subscriber('ardrone/sensor_fusion/pose', QuadrotorPose, callback = self.recieve_estimated_velocity),
			desired_pose = rospy.Subscriber('ardrone/trajectory/pose', QuadrotorPose, callback = self.recieve_desired_pose),
			desired_velocity = rospy.Subscriber('ardrone/trajectory/velocity', QuadrotorPose, callback = self.recieve_desired_velocity),
			controller_state = rospy.Subscriber('ardrone/controller_state', ControllerStateMsg, callback = self.recieve_state),
		)

		self.commander = QuadrotorCommands.Commands()

		self.timer = dict( 
			publisher = rospy.Timer(rospy.Duration( COMMAND_TIME ), self.publish_twist, oneshot=False) 
			)

		now_time = rospy.get_time()
		
		self.callback_time = dict(
			recieve_estimated_pose = now_time, 
			recieve_estimated_velocity = now_time, 
			recieve_desired_pose = now_time, 
			recieve_desired_velocity = now_time, 
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
		default = dict(x = dict(P = 1.0, D = 0.0, I = 0.0), y = dict(P = 1.0, D = 0.0, I = 0.0), z= dict(P = 1.0, D = 0.0, I = 0.0), yaw = dict(P = 1.0, D = 0.0, I = 0.0) )
		PID_PARAMS = rospy.get_param( '/controller/PID_Gains', default )
		self.controller = dict()
		for key, controller_dict in PID_PARAMS.items():
			self.controller[ key.lower() ] = Controllers.PID( 
				Kp = controller_dict['P'],
				Ki = controller_dict['I'],
				Kd = controller_dict['D'],
				)
		self.controller['yaw'].periodic = True

		rospy.logwarn("\nController Configuration Inited by Default to:\nX Controller is type PID with {0} \nY Controller is type PID with {1}\nZ Controller is type PID with {2} \nYaw Controller is type PID with {3} ".format(
			str(self.controller['x']), str(self.controller['y']), str(self.controller['z']), str(self.controller['yaw']) ) )

		self.controller_state = False

	def update_callback_time( self, callback_name ):
		aux_time = rospy.get_time()
		dt = aux_time - self.callback_time[ callback_name ]
		self.callback_time[ callback_name ] = aux_time
		return dt 

	def recieve_estimated_pose( self, pose ):
		dt = self.update_callback_time( inspect.stack()[0][3] )
		for key, siso_controller in self.controller.items():
			siso_controller.input_pose( getattr(pose, key), dt )
			
		self.position['yaw'] = getattr(pose, 'yaw')

	def recieve_estimated_velocity( self, velocity ):
		#dt = self.update_callback_time( inspect.stack()[0][3] )
		for key, siso_controller in self.controller.items():
			try:
				siso_controller.input_velocity( getattr(velocity, key) )
			except AttributeError:
				pass 

	def recieve_desired_pose( self, pose ):
		#dt = self.update_callback_time( inspect.stack()[0][3] )
		for key, siso_controller in self.controller.items():
			siso_controller.change_set_point_pose( getattr(pose, key) )

	def recieve_desired_velocity( self, velocity ):
		#dt = self.update_callback_time( inspect.stack()[0][3] )

		for key, siso_controller in self.controller.items():
			self.velocity[key] = getattr(velocity, key)
			try:
				siso_controller.change_set_point_velocity( getattr(velocity, key) )
			except AttributeError:
				pass
			
	def recieve_state( self, controller_state ):
		self.controller_state.set_state( controller_state ) 
		rospy.logwarn( 'Controller succesfully {0}activated!'.format( '' if self.controller_state.on else 'de' ) )
		if self.controller_state.on:
			rospy.logwarn( '{0} Control'.format('position' if self.controller_state.position else 'velocity'))

	def publish_twist( self, time ):
		if not self.controller_state.on:
			return
		else:
			velocity = dict()
			if self.controller_state.position:
				for key, siso_controller in self.controller.items():
					velocity[key] = siso_controller.get_output()
					if abs(velocity[key]) < 0.005: #go-to-Hover
						velocity[key] = 0.00

				yaw = self.position['yaw']
				vx =  velocity['x'] * math.cos( yaw ) + velocity['y'] * math.sin( yaw )
				vy =  - velocity['x'] * math.sin( yaw ) + velocity['y'] * math.cos( yaw )

				velocity['x'] = vx
				velocity['y'] = vy 
			else:
				velocity = self.velocity 

			
			"""
			if abs(velocity['x']) > 0.1:
				velocity['x'] = 0.1 if velocity['x']>0 else -0.1
			if abs(velocity['y']) > 0.1:
				velocity['y'] = 0.1 if velocity['y']>0 else -0.1
			"""
			self.commander.velocity( velocity )


def main():
	rospy.init_node('ardrone_controller', anonymous = True)
	Controller()
	rospy.spin()

if __name__ == "__main__": main()