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
from std_msgs.msg import Bool 
from ardrone_autonomy.msg import Navdata 
from sensor_msgs.msg import Joy 

from ardrone_control.srv import OpenLoopCommand 

import tf

import sys
from PyQt4 import QtGui, QtCore


from math import pi , cos, sin 

STEP = dict( 
	x = 0.1, 
	y = 0.1, 
	z = 0.1, 
	yaw = pi/36.0)
COMMAND_TIME = 0.01

class ArdroneCommander(Quadrotor, object):
	"""docstring for ArdroneCommander"""
	def __init__(self, **kwargs):
		super(ArdroneCommander, self).__init__( **kwargs )
		self.commander = Commands()
		
		#self.publisher_frame = 'local'
		
		self.signal = [ ]
		
		self.publisher = dict( 
			controller_state = rospy.Publisher('ardrone/controller_state', Bool, latch = True),
			trajectory = rospy.Publisher('ardrone/desired_pose', Odometry)
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
		
		# self.tfListener = tf.TransformListener()

		self.timer = dict( 
			trajectory = rospy.Timer(rospy.Duration( COMMAND_TIME ), self.talk, oneshot=False) 
			)

		self.controller_state  = False #on or off 

	def talk( self, time_data ):
		if self.controller_state:
			self.publish_goal()
		elif self.state == ArDroneStates.Flying or self.state == ArDroneStates.Hovering: #flying or hovering
			self.commander.velocity( self.velocity ) 
			
	def publish_controller_state( self, boolean ):
		msg = Bool()
		msg.data = boolean 
		self.publisher['controller_state'].publish(msg)

	def publish_goal( self ):
		msg = Odometry( )
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "/nav"
		msg.child_frame_id = "/goal" 

		# position
		msg.pose.pose.position.x = self.position['x']
		msg.pose.pose.position.y = self.position['y']
		msg.pose.pose.position.z = self.position['z']

		# orientation
		msg.pose.pose.orientation.x = self.orientation.x
		msg.pose.pose.orientation.y = self.orientation.y
		msg.pose.pose.orientation.z = self.orientation.z
		msg.pose.pose.orientation.w = self.orientation.w
		
		# linear velocity
		msg.twist.twist.linear.x = self.velocity['x']
		msg.twist.twist.linear.y = self.velocity['y']
		msg.twist.twist.linear.z = self.velocity['z']

		# angular velocity
		msg.twist.twist.angular.x = self.velocity['roll']
		msg.twist.twist.angular.y = self.velocity['pitch']
		msg.twist.twist.angular.z = self.velocity['yaw']

		self.publisher['trajectory'].publish(msg)

		self.tf_broadcaster['goal_tf'].sendTransform( 
			(self.position['x'], self.position['y'], self.position['z']) , 
            self.orientation.get_quaternion(),
            msg.header.stamp, 
            msg.child_frame_id, 
            msg.header.frame_id)

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
		rospy.logwarn("Reset Drone!")
		self.commander.reset()

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
		rospy.set_param('/ardrone_autonomy/do_imu_calibration', "true")
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
		#self.position['yaw'] = navdata.rotZ * pi / 180.0 

	def change_set_point( self, direction, scale ):
		self.position[direction] += scale * STEP[ direction ]
	
	def x( self, scale ):
		if self.controller_state:
			self.change_set_point( 'x', scale )
		else:
			self.velocity['x'] = scale

	def y( self, scale ):
		if self.controller_state:
			self.change_set_point( 'y', scale )
		else:
			self.velocity['y'] = scale;

	def z( self, scale ):
		if self.controller_state:
			self.change_set_point( 'z', scale )
		else:
			self.velocity['z'] = scale;

	def yaw( self, scale ):
		if self.controller_state:
			self.change_set_point( 'yaw', scale )
			if self.position['yaw'] >  pi :
				self.position['yaw'] -= 2 * pi;
			if self.position['yaw'] <= -pi:
				self.position['yaw'] += 2 * pi;

			#orientation changes over CHANGED yaw angle
			self.orientation.set_euler( dict( yaw=self.position['yaw'], pitch = 0.0, roll = 0.0 ) )

		else:
			self.velocity['yaw'] = scale;

	def control_on( self, *args ):
		rospy.logwarn("Control Activated")
		self.controller_state = True 
		self.publish_controller_state( self.controller_state )

	def control_off( self, *args ):
		rospy.logwarn("Control Deactivated")
		self.controller_state = False 
		self.publish_controller_state( self.controller_state )

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
		lbl_x = QtGui.QLabel("Press IK to +- X", self)
		lbl_y = QtGui.QLabel("Press JL to +- Y", self)
		lbl_z = QtGui.QLabel("Press WS to +- Z", self)
		lbl_yaw = QtGui.QLabel("Press AD to +- Yaw", self)

		lbl_take_off =  QtGui.QLabel("Press Return to Take Off", self)
		lbl_land =  QtGui.QLabel("Press Return to Landing", self)
		lbl_reset =  QtGui.QLabel("Press Return to Reset", self)

		lbl_position =  QtGui.QLabel("Press C to control position ", self)
		#lbl_local =  QtGui.QLabel("Press V to control velocity in local frame", self)
		#lbl_global =  QtGui.QLabel("Press G to control velocity in global frame", self)

		lbl_chirp = QtGui.QLabel("Press 1, 2, 3, 4 to command a chirp in X, Y, Z, Yaw)", self)

		lbl_cal = QtGui.QLabel("Press Z to Flat Trim and X to Calibrate Imu", self)
		
		self.resize(1000, 600)
		self.center()

		vbox = QtGui.QVBoxLayout()
		vbox.addWidget(lbl_x); vbox.addWidget(lbl_y); vbox.addWidget(lbl_z); vbox.addWidget(lbl_yaw);
		vbox.addWidget(lbl_take_off); vbox.addWidget(lbl_reset); vbox.addWidget(lbl_land); 
		vbox.addWidget(lbl_position); #vbox.addWidget(lbl_local);  vbox.addWidget(lbl_global); 
		vbox.addWidget(lbl_chirp); vbox.addWidget(lbl_cal);
		self.setLayout(vbox)
		self.setWindowTitle('Ar.Drone KeyBoard Control') 
		QtGui.QApplication.setStyle(QtGui.QStyleFactory.create('Cleanlooks')) 
		#self.setWindowIcon(QtGui.QIcon('figs/parrot_ar_drone.jpg'))   
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
		if key == QtCore.Qt.Key_A:
			self.yaw( -1 )
		elif key == QtCore.Qt.Key_D:
			self.yaw( +1 )
		elif key == QtCore.Qt.Key_S:
			self.z( -1 )
		elif key == QtCore.Qt.Key_W:
			self.z( +1)
		elif key == QtCore.Qt.Key_I:
			self.x( -1 )
		elif key == QtCore.Qt.Key_K:
			self.x( +1 )
		elif key == QtCore.Qt.Key_L:
			self.y( -1 )
		elif key == QtCore.Qt.Key_J:
			self.y( +1 )
		elif key == QtCore.Qt.Key_Return:
			self.take_off()
		elif key == QtCore.Qt.Key_Backspace:
			self.land()
		elif key == QtCore.Qt.Key_Space:
			self.reset()
		elif key == QtCore.Qt.Key_C:
			self.control_on()
		elif key == QtCore.Qt.Key_F:
			self.control_off()
			#self.local_command_frame()			
		elif key == QtCore.Qt.Key_G:
			self.control_off()
			#self.global_command_frame()	
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
	command_method = rospy.get_param("ardrone_commander/command_method")
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