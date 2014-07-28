#!/usr/bin/env python  
# -*- coding: utf-8 -*-

from lib.QuadrotorCommands import Commands
from lib.Quadrotor import Quadrotor, ArDroneStates 
from lib.Joysticks import PS2, PS3 
from lib.Signals import SignalResponse
import rospy 

from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Vector3Stamped, Twist 
#from diagnostic_msgs.msg import KeyValue
#from std_msgs.msg import Bool 
from ardrone_autonomy.msg import Navdata 
from sensor_msgs.msg import Joy 

from ardrone_control.srv import OpenLoopCommand 
from ardrone_control.msg import QuadrotorPose
from ardrone_control.msg import ControllerState

import tf

import sys
from PyQt4 import QtGui, QtCore
import inspect


#from math import pi , cos, sin 
import math
STEP = dict( 
	x = 0.1, 
	y = 0.1, 
	z = 0.1, 
	yaw = math.pi/36.0)
COMMAND_TIME = 0.01

class ArdroneCommander(Quadrotor, object):
	"""docstring for ArdroneCommander"""
	def __init__(self, **kwargs):
		super(ArdroneCommander, self).__init__( **kwargs )
		self.commander = Commands()
				
		self.signal = [ ]
		
		self.publisher = dict( 
			controller_state = rospy.Publisher('ardrone/controller_state', ControllerState, latch = True),
			desired_pose = rospy.Publisher('ardrone/trajectory/pose', QuadrotorPose),
			desired_velocity = rospy.Publisher('ardrone/trajectory/velocity', QuadrotorPose),
			)

		self.subscriber = dict( 
			ardrone_state = rospy.Subscriber('ardrone/navdata', Navdata, callback = self.recieve_navdata),
			)

		self.service = dict(
			signal = rospy.Service('ardrone_control/Signal', OpenLoopCommand, self.signal_init)
			)

		self.tf_broadcaster = dict( 
			goal_tf = tf.TransformBroadcaster() 
			)
		
		self.timer = dict( 
			trajectory = rospy.Timer(rospy.Duration( COMMAND_TIME ), self.talk, oneshot=False) 
			)

		self.controller_state  = False #on or off 

	def talk( self, time_data ):
		if self.state == ArDroneStates.Flying or self.state == ArDroneStates.Hovering:
			self.publisher['desired_pose'].publish( self.get_msg(self.position) )
			self.publisher['desired_velocity'].publish( self.get_msg(self.velocity) ) 
		else:
			self.publisher['desired_velocity'].publish( self.get_msg( dict(x = 0.0, y=0.0, z = 0.0, yaw = 0.0) ) )
			
	def publish_controller_state( self ):
		msg = ControllerState()

		msg.on = (self.state == ArDroneStates.Flying or self.state == ArDroneStates.Hovering)
		msg.position = self.controller_state

		rospy.logwarn("{0} Controller".format('Activating' if msg.on else 'Deactivating'))
		if self.controller_state:
			rospy.logwarn('Setting {0} Control'.format( 'position' if msg.position else 'velocity') )
		self.publisher['controller_state'].publish(msg)

	def get_msg( self, attribute ):
		msg = QuadrotorPose()
		for key, value in attribute.items():
			setattr(msg, key, value)

		return msg 

	def outdoor( self ):
		rospy.set_param('/ardrone_autonomy/outdoor', 1)

	def indoor( self ):
		rospy.set_param('/ardrone_autonomy/outdoor', 0)

	def fly_with_shell( self ):
		rospy.set_param('/ardrone_autonomy/flight_without_shell', 0)

	def fly_without_shell( self ):
		rospy.set_param('/ardrone_autonomy/flight_without_shell', 1)
		
	def take_off( self, *args ):
		if self.state == ArDroneStates.Landed : #landed
			rospy.logwarn("Take Off Drone!")
			self.commander.take_off()
		else:
			rospy.logwarn("Drone is not landed")

	def reset( self, *args ):
		if self.state == ArDroneStates.Landed or self.state == ArDroneStates.Unknown: #landed
			rospy.logwarn("Reset Drone!")
			self.commander.reset()
		else:
			rospy.logwarn("It is not safe to reset!")

	def land( self, *args ):
		rospy.logwarn("Land Drone!")
		self.commander.land() 

	def stop( self, *args ):
		self.control_off()
		for key in self.velocity.keys():
			self.velocity[key] = 0.0
		rospy.logwarn("Stop!")
		self.commander.velocity( self.velocity )

	def toggle_cam( self, *args ):
		self.commander.toggle_cam() 

	def cam_select( self, cam ):
		self.commander.cam_select( cam )

	def imu_calibration( self, *args ):
		#rospy.set_param('/ardrone_autonomy/do_imu_caliberation', True)
		self.commander.imu_calibration( )

	def flat_trim( self, *args ):
		self.commander.flat_trim( )

	def record( self, enable ):
		self.commander.record( enable )

	def led_animation( self, animation_type, freq, duration ):
		self.commander.led_animation( animation_type, freq, duration )

	def flight_animation( self, animation_type, duration):
		self.commander.flight_animation( animation_type, duration )

	def recieve_navdata( self, navdata ):
		self.set_state( navdata.state )
		self.battery = navdata.batteryPercent

	def change_set_point( self, direction, scale ):
		self.position[direction] += scale * STEP[ direction ]
	
	def change_velocity( self, direction, scale):
		self.velocity[direction] = scale * STEP[direction]

	def x( self, scale ):
		self.change_set_point( 'x', scale )

	def y( self, scale ):
		self.change_set_point( 'y', scale )

	def z( self, scale ):
		self.change_set_point( 'z', scale )

	def yaw( self, scale ):
		self.change_set_point( 'yaw', scale )
		self.position['yaw'] = math.atan2( math.sin(self.position['yaw']), math.cos(self.position['yaw']) )
		self.orientation.set_euler( dict( yaw=self.position['yaw'], pitch = 0.0, roll = 0.0 ) )

	def vx( self, scale ):
		self.change_velocity( 'x', scale )

	def vy( self, scale ):
		self.change_velocity( 'y', scale )

	def vz( self, scale ):
		self.change_velocity( 'z', scale )

	def vyaw( self, scale ):
		self.change_velocity( 'yaw', scale )

	def control_on( self, *args ):
		self.controller_state = True 
		self.publish_controller_state( )

	def control_off( self, *args ):
		self.controller_state = False 
		self.publish_controller_state( )

	def signal_init( self, signal_data ):
		# inits signal
		if len(self.signal):
			rospy.logwarn( "It's still sending data" )
		else:
			if self.state == ArDroneStates.Flying or self.state == ArDroneStates.Hovering: 
				#self.control_off()
				self.stop()
				self.signal = SignalResponse( tf = signal_data.time, dt = signal_data.dt, f = signal_data.f, signal = signal_data.signal, direction = signal_data.direction ) 
				self.timer['signal'] = rospy.Timer( rospy.Duration(self.signal.dt), self.cmd_signal, oneshot = False )
				rospy.logwarn("Signal Started!")
			else:
				rospy.logwarn(  "Drone not Flying" )
	
	def cmd_signal( self, time_data ):
		# commands signal 			
		if len( self.signal ):
			self.velocity[ self.signal.direction ] = self.signal.command()
		else:
			self.signal_off()

	def signal_off( self ):
		# ends signal
		try:
			self.timer['signal'].shutdown( )
			self.signal = [ ]
			rospy.logwarn("Signal Finished!")
		except AttributeError:
			pass

		self.stop()

	def default_chirp_data(self, direction):
		data = OpenLoopCommand()
		data.time = 10
		data.dt = 0.01
		data.f = 20
		data.signal = 'chirp'
		data.direction = direction

		return data

	def chirp_x( self, *args ):
		self.signal_init( self.default_chirp_data('x') ) 

	def chirp_y( self, *args ):
		self.signal_init( self.default_chirp_data('y') ) 

	def chirp_z( self, *args ):
		self.signal_init( self.default_chirp_data('z') ) 

	def chirp_yaw( self, *args ):
		self.signal_init( self.default_chirp_data('yaw') )  

class JoystickController(ArdroneCommander, object):
	"""docstring for JoystickController"""

	def __init__(self, **kwargs):
		super(JoystickController, self).__init__()
		rospy.Subscriber('/joy', Joy, callback = self.recieve_joy)

		self.joy = kwargs.get( 'joy', [] )

	def recieve_joy(self, joy_data):
		try:
			for button, data_idx in self.joy.BUTTONS.items():
				if joy_data.buttons[ data_idx ]:
					try:
						getattr( self, self.joy.COMMAND[button] )()
					except KeyError:
						pass

			for axis, data_idx in self.joy.AXES.items():
				try: 
					getattr( self, self.joy.COMMAND[axis] )( joy_data.axes[ data_idx ] )
				except KeyError:
					pass
		except AttributeError:
			pass

class KeyBoardController(ArdroneCommander, QtGui.QWidget, object):
	"""docstring for KeyBoardC"""
	def __init__(self, **kwargs):
		super(KeyBoardController, self).__init__()
		self.initUI()

	def initUI( self ):
		main_groupBox = QtGui.QGroupBox("Key Press Info:")

		group_list = ['position', 'velocity', 'direct', 'control', 'chirp']
		coordinate_list = ['x', 'y', 'z', 'yaw']
		labels = dict()
		labels['velocity'] = dict()
		labels['velocity']['x'] = QtGui.QLabel("IK")
		labels['velocity']['y'] = QtGui.QLabel("LJ")
		labels['velocity']['z'] = QtGui.QLabel("WS")
		labels['velocity']['yaw'] = QtGui.QLabel("DA")

		labels['position'] = dict()
		labels['position']['x'] = QtGui.QLabel("Up-Down")
		labels['position']['y'] = QtGui.QLabel("Right-Left")
		labels['position']['z'] = QtGui.QLabel("Up-Down+Shift")
		labels['position']['yaw'] = QtGui.QLabel("Right-Left+Shift")

		labels['direct'] = dict()
		labels['direct']['take_off'] = QtGui.QLabel("Return")
		labels['direct']['land'] = QtGui.QLabel("Backspace")
		labels['direct']['reset'] = QtGui.QLabel("Spacebar")
		labels['direct']['stop'] = QtGui.QLabel("0")

		labels['control'] = dict()
		labels['control']['position'] = QtGui.QLabel("C")
		labels['control']['velocity'] = QtGui.QLabel("F")
		labels['control']['flat_trim'] = QtGui.QLabel('Z')
		labels['control']['imu_recalibrate'] = QtGui.QLabel('X')

		labels['chirp'] = dict()
		labels['chirp']['x'] = QtGui.QLabel("1")
		labels['chirp']['y'] = QtGui.QLabel("2")
		labels['chirp']['z'] = QtGui.QLabel("3")
		labels['chirp']['yaw'] = QtGui.QLabel("4")

		layout = QtGui.QVBoxLayout()
		for group in group_list:
			other = False
			sub_group = QtGui.QGroupBox(group)
			sub_layout = QtGui.QGridLayout()
			
			i = 0
			for coordinate in coordinate_list:
				try:
					sub_layout.addWidget( labels[group][coordinate], 1, i )
					sub_layout.addWidget( QtGui.QLabel("<b>{0}</b>".format(coordinate)), 0, i )
				except KeyError:
					other = True 
				i += 1
			if other:
				i = 0
				for key, label in labels[group].items():
					sub_layout.addWidget( labels[group][key], 1, i )
					sub_layout.addWidget( QtGui.QLabel("<b>{0}</b>".format(key)), 0, i )
					i += 1


			sub_group.setLayout(sub_layout)
			layout.addWidget(sub_group)

		main_groupBox.setLayout( layout )

		main_layout = QtGui.QVBoxLayout()
		main_layout.addWidget(main_groupBox)
		self.setLayout(main_layout)


		self.setWindowTitle('Ar.Drone KeyBoard Control') 
		QtGui.QApplication.setStyle(QtGui.QStyleFactory.create('Cleanlooks')) 
		self.setWindowIcon(QtGui.QIcon('figs/parrot_ar_drone.jpg'))   
		self.show()

	def center(self):
		qr = self.frameGeometry()
		cp = QtGui.QDesktopWidget().availableGeometry().center()
		qr.moveCenter(cp)
		self.move(qr.topLeft())

	def keyPressEvent(self, event):
		key = event.key()
		if event.isAutoRepeat():
			return
		
		## Global Commands
		if key == QtCore.Qt.Key_Return:
			self.take_off()
		elif key == QtCore.Qt.Key_Backspace:
			self.land()
		elif key == QtCore.Qt.Key_Space:
			self.reset()
		elif key == QtCore.Qt.Key_0:
			self.stop()
		elif key == QtCore.Qt.Key_C:
			self.control_on()
		elif key == QtCore.Qt.Key_F:
			self.control_off()
		elif key == QtCore.Qt.Key_G:
			self.control_off()
		elif key == QtCore.Qt.Key_1:
			self.chirp_x()	
		elif key == QtCore.Qt.Key_2:
			self.chirp_y()	
		elif key == QtCore.Qt.Key_3:
			self.chirp_z()	
		elif key == QtCore.Qt.Key_4:
			self.chirp_yaw()	
		elif key == QtCore.Qt.Key_T:
			self.toggle_cam()	
		elif key == QtCore.Qt.Key_Z:
			self.flat_trim()	
		elif key == QtCore.Qt.Key_X:
			self.imu_calibration()

		## Velocity Commands
		if key == QtCore.Qt.Key_A:
			self.vyaw( -1 )
		elif key == QtCore.Qt.Key_D:
			self.vyaw( +1 )
		elif key == QtCore.Qt.Key_S:
			self.vz( -1 )
		elif key == QtCore.Qt.Key_W:
			self.vz( +1)
		elif key == QtCore.Qt.Key_I:
			self.vx( -1 )
		elif key == QtCore.Qt.Key_K:
			self.vx( +1 )
		elif key == QtCore.Qt.Key_L:
			self.vy( -1 )
		elif key == QtCore.Qt.Key_J:
			self.vy( +1 )

		## Position Commands
		modifiers = QtGui.QApplication.keyboardModifiers()
		if modifiers == QtCore.Qt.ShiftModifier:
			if key == QtCore.Qt.Key_Left:
				self.yaw( -1 )
			elif key == QtCore.Qt.Key_Right:
				self.yaw( +1 )
			elif key == QtCore.Qt.Key_Down:
				self.z( -1 )
			elif key == QtCore.Qt.Key_Up:
				self.z( +1 )
		else:
			if key == QtCore.Qt.Key_Left:
				self.y( -1 )
			elif key == QtCore.Qt.Key_Right:
				self.y( +1 )
			elif key == QtCore.Qt.Key_Down:
				self.x( -1 )
			elif key == QtCore.Qt.Key_Up:
				self.x( +1 )

	def keyReleaseEvent(self, event):
		key = event.key()
		if event.isAutoRepeat():
			return

		if key == QtCore.Qt.Key_A or key == QtCore.Qt.Key_D:
			self.yaw( 0 )
		elif key == QtCore.Qt.Key_S or key == QtCore.Qt.Key_W:
			self.z( 0 )
		elif key == QtCore.Qt.Key_I or key == QtCore.Qt.Key_K:
			self.x( 0 )
		elif key == QtCore.Qt.Key_L or key == QtCore.Qt.Key_J:
			self.y( 0 )

	def closeEvent(self, event):
		reply = QtGui.QMessageBox.question(self, 'Message',
			"Are you sure to quit?", QtGui.QMessageBox.Yes | 
			QtGui.QMessageBox.No, QtGui.QMessageBox.No)

		if reply == QtGui.QMessageBox.Yes:
			event.accept()
			self.land()
		else:
			event.ignore() 
		
def main():
	rospy.init_node('ardrone_commander', anonymous = True)
	command_method = rospy.get_param("ardrone_commander/command_method", '')
	if command_method == 'PS3':
		commander = JoystickController( joy = PS3() )
	elif command_method == 'PS2':
		commander = JoystickController( joy = PS2() )
	else:
		app = QtGui.QApplication(sys.argv)
		commander = KeyBoardController( )
		sys.exit(app.exec_())


	rospy.spin()

if __name__ == "__main__": main()