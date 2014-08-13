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

import scipy.signal
import inspect
import math 
#from tf import TransformListener
COMMAND_TIME = 0.01
MIN_ERROR_DICT = dict( 
	x = 0.2,
	y = 0.2,
	z = 0.1,
	yaw = 5.0 * math.pi/180.0)

MAX_ERROR_DICT = dict( 
	x = 0.4,
	y = 0.4,
	z = 0.2,
	yaw = 20.0 * math.pi/180.0)

MIN_ERROR = 0.3
MAX_ERROR = 0.6

class Controller(Quadrotor, object):
	"""docstring for Controller"""
	def __init__(self, **kwargs):
		super(Controller, self).__init__(**kwargs)
		self.default_init()
		self.controller_state = Controllers.ControllerState()
		self.timer = dict( 
			publisher = rospy.Timer(rospy.Duration( COMMAND_TIME ), self.publish_twist, oneshot=False) 
			)
		
		self.subscriber = dict(
			estimated_pose = rospy.Subscriber('ardrone/sensor_fusion/pose', QuadrotorPose, callback = self.recieve_estimated_pose),
			estimated_velocity = rospy.Subscriber('ardrone/sensor_fusion/pose', QuadrotorPose, callback = self.recieve_estimated_velocity),
			desired_pose = rospy.Subscriber('ardrone/trajectory/pose', QuadrotorPose, callback = self.recieve_desired_pose),
			desired_velocity = rospy.Subscriber('ardrone/trajectory/velocity', QuadrotorPose, callback = self.recieve_desired_velocity),
			controller_state = rospy.Subscriber('ardrone/controller_state', ControllerStateMsg, callback = self.recieve_state),
		)

		self.commander = QuadrotorCommands.Commands()

		now_time = rospy.get_time()
		
		self.callback_time = dict(
			recieve_estimated_pose = now_time, 
			recieve_estimated_velocity = now_time, 
			recieve_desired_pose = now_time, 
			recieve_desired_velocity = now_time, 
			)

		self.error = 0

		self.srv = Server(ControllerConfig, self.configure_controller )
		#self.tfListener = tf.TransformListener()
	
	def configure_controller(self, config, level):
		controller_type = config.type 
		controller_direction = config.direction
		digital = config.digital 
		Ts = config.Ts

		if config.direction in self.controller.keys():

			changed = True
			if 'PID' in controller_type:
				Kp = config.proportional
				Kd = config.derivative
				Ki = config.integral
				if not digital:
					Kp = Kp - Ki / 2
					
				self.controller[controller_direction] = getattr(Controllers, controller_type)( 
					Kp = Kp,
					Kd = Kd,
					Ki = Ki, 
					periodic = (controller_direction == 'yaw')
					)
			elif 'Zero-Pole Gain' in controller_type:
				try:
					zeros = [float(a) for a in config.zeros.split(',')]
				except ValueError:
					zeros = []
				try:
					poles = [float(a) for a in config.poles.split(',')]
				except ValueError:
					poles = [ ]

				gain = float( config.gain )
				if not digital:
					if (zeros != list() or poles != list()):
						zeros, poles, gain, Ts = scipy.signal.cont2discrete( (zeros, poles, gain), Ts , "bilinear")

				self.controller[controller_direction] = Controllers.ZPK( 
					zeros,
					poles,
					gain = float( gain ), 
					periodic = (controller_direction == 'yaw')
					)
			elif 'Transfer Function' in controller_type:
				try:
					numerator = [float(a) for a in config.numerator.split(',')]
				except ValueError:
					numerator = []
				try:
					denominator = [float(a) for a in config.denominator.split(',')]
				except ValueError:
					denominator = []

				
				
				if not digital:
					numerator, denominator, Ts = scipy.signal.cont2discrete( (numerator, denominator), Ts , "bilinear")

				self.controller[controller_direction] = Controllers.TF( 
					numerator,
					denominator,
					periodic = (controller_direction == 'yaw')
					)
			else:
				changed = False

			if changed:
				rospy.loginfo("\nController Configuration Changed! \nFor Safety Reasons it will Deactivate, please reactivate manually!")
				rospy.loginfo('\nNew Controller is {0} in direction {1}'.format(config.type, config.direction))
				rospy.loginfo('\n Controller has {0}'.format( str(self.controller[config.direction]) ))
				self.controller_state.on = False


				self.timer['publisher'].shutdown()
				self.timer['publisher'] =  rospy.Timer( rospy.Duration( Ts ), self.publish_twist, oneshot=False)
			
		return config 

	def default_init(self):
		#self.controller_state = False
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

		rospy.loginfo("\nController Configuration Inited by Default to:\nX Controller is type PID with {0} \nY Controller is type PID with {1}\nZ Controller is type PID with {2} \nYaw Controller is type PID with {3} ".format(
			str(self.controller['x']), str(self.controller['y']), str(self.controller['z']), str(self.controller['yaw']) ) )

		#self.controller_state = False

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

	def recieve_estimated_velocity( self, local_velocity ):
		#dt = self.update_callback_time( inspect.stack()[0][3] )

		global_velocity = self.local_to_global( local_velocity )
		for key, siso_controller in self.controller.items():
			try:
				siso_controller.input_velocity( getattr(global_velocity, key) )
			except AttributeError:
				pass 

	def recieve_desired_pose( self, pose ):
		#dt = self.update_callback_time( inspect.stack()[0][3] )
		for key, siso_controller in self.controller.items():
			siso_controller.change_set_point_pose( getattr(pose, key) )

	def recieve_desired_velocity( self, local_velocity ):
		#dt = self.update_callback_time( inspect.stack()[0][3] )

		global_velocity = self.local_to_global( local_velocity )
		for key, siso_controller in self.controller.items():
			self.velocity[key] = getattr(local_velocity, key)
			try:
				#pass
				siso_controller.change_set_point_velocity( getattr(global_velocity, key) )
			except AttributeError:
				pass
	
	def local_to_global( self, attribute ):
		aux = type(attribute)()
		yaw = self.position['yaw']

		if type(attribute) == dict:
			aux = dict()
			aux['x'] =  attribute['x'] * math.cos( yaw ) - attribute['y'] * math.sin( yaw )
			aux['y'] =  attribute['x'] * math.sin( yaw ) + attribute['y'] * math.cos( yaw )
			aux['z'] =  attribute['z']
			aux['yaw'] =  attribute['yaw']

		elif type(attribute) == QuadrotorPose:
			aux = QuadrotorPose()
			aux.x = attribute.x * math.cos( yaw ) - attribute.y * math.sin( yaw )
			aux.y = attribute.x * math.sin( yaw ) + attribute.y * math.cos( yaw )
			aux.z = attribute.z 
			aux.yaw = attribute.yaw 

		return aux 

	def global_to_local( self, attribute ):
		aux = type(attribute)()
		yaw = self.position['yaw']

		if type(attribute) == dict:
			aux = dict()
			aux['x'] =  attribute['x'] * math.cos( yaw ) + attribute['y'] * math.sin( yaw )
			aux['y'] =  - attribute['x'] * math.sin( yaw ) + attribute['y'] * math.cos( yaw )
			aux['z'] =  attribute['z']
			aux['yaw'] =  attribute['yaw']

		elif type(attribute) == QuadrotorPose:
			aux = QuadrotorPose()
			aux.x = attribute.x * math.cos( yaw ) + attribute.y * math.sin( yaw )
			aux.y = - attribute.x * math.sin( yaw ) + attribute.y * math.cos( yaw )
			aux.z = attribute.z 
			aux.yaw = attribute.yaw 

		return aux 

	def recieve_state( self, controller_state ):
		self.controller_state.set_state( controller_state ) 
		"""
		rospy.loginfo( 'Controller succesfully {0}activated!'.format( '' if self.controller_state.on else 'de' ) )
		if self.controller_state.on:
			if controller_state.position:
				rospy.loginfo( 'Position Control')
			elif self.controller_state.local:
				rospy.loginfo( 'Local Velocity Control' )
			else:
				rospy.loginfo( 'Global Velocity Control' )
		"""

	def publish_twist( self, time ):
		if not self.controller_state.on:
			return
		else:
			velocity = dict()
			aux = 0
			if self.controller_state.position:
				for key, siso_controller in self.controller.items():
					velocity[key] = siso_controller.get_output()
					aux += abs(siso_controller.get_error())

				velocity = self.global_to_local( velocity )
				"""
				yaw = self.position['yaw']
				vx =  velocity['x'] * math.cos( yaw ) + velocity['y'] * math.sin( yaw )
				vy =  - velocity['x'] * math.sin( yaw ) + velocity['y'] * math.cos( yaw )

				velocity['x'] = vx
				velocity['y'] = vy 
				"""
				if self.controller_state.flying:
					if aux < MIN_ERROR:
						for key in velocity.keys():
							velocity[key] = 0 #go-to-hover
				elif self.controller_state.hovering:
					if aux < MAX_ERROR:
						for key in velocity.keys():
							velocity[key] = 0 #stay-hovering
			else:
				velocity = self.velocity 

				if not self.controller_state.local:
					yaw = self.position['yaw']
					vx =  velocity['x'] * math.cos( yaw ) + velocity['y'] * math.sin( yaw )
					vy =  - velocity['x'] * math.sin( yaw ) + velocity['y'] * math.cos( yaw )

					velocity['x'] = vx
					velocity['y'] = vy 

			self.commander.velocity( velocity )

def main():
	rospy.init_node('ardrone_controller', anonymous = True)
	Controller()
	rospy.spin()

if __name__ == "__main__": main()