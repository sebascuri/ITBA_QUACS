#!/usr/bin/env python  
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('ardrone_control')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.srv import CamSelect, FlightAnim, LedAnim, RecordEnable 
import std_srvs.srv

class Commands(object):
	"""docstring for Commands"""
	def __init__(self):
		super(Commands, self).__init__()

		self.publisher = dict( 
			land = rospy.Publisher('ardrone/land', Empty, latch = True),
			take_off = rospy.Publisher('ardrone/takeoff', Empty, latch = True),
			reset = rospy.Publisher('ardrone/reset', Empty, latch = True),
			cmd_vel = rospy.Publisher('cmd_vel', Twist)
			)

		self.service = dict( 
			toggle_cam = rospy.ServiceProxy('ardrone/togglecam', std_srvs.srv.Empty),
			cam_select = rospy.ServiceProxy('ardrone/setcamchannel', CamSelect),
			led_animation = rospy.ServiceProxy( 'ardrone/setledanimation', LedAnim),
			imu_calibration = rospy.ServiceProxy('ardrone/imu_recalib', std_srvs.srv.Empty),
			flat_trim = rospy.ServiceProxy('ardrone/flattrim', std_srvs.srv.Empty),
			record = rospy.ServiceProxy('ardrone/setrecord', RecordEnable),
			flight_animation = rospy.ServiceProxy('ardrone/setflightanimation', FlightAnim),
			)
		
	def take_off( self ):
		self.publisher['take_off'].publish()

	def reset( self ):
		self.publisher['reset'].publish()

	def land( self ):
		self.publisher['land'].publish()

	def velocity(self, velocity_dict):
		msg = Twist() 

		msg.linear.x = velocity_dict.get('x', 0.0)
		msg.linear.y = velocity_dict.get('y', 0.0)
		msg.linear.z = velocity_dict.get('z', 0.0)
		msg.angular.z = velocity_dict.get('yaw', 0.0)

		#msg.angular.y = velocity_dict.get('pitch', 1.0)
		#msg.angular.x = velocity_dict.get('roll', 1.0)

		#disable auto_hover mode
		msg.angular.y = velocity_dict.get('pitch', 1.0) #1.0
		msg.angular.x = velocity_dict.get('roll', 1.0) #1.0

		self.publisher['cmd_vel'].publish( msg )

	def toggle_cam( self ):
		rospy.wait_for_service('ardrone/togglecam')
		try:
			self.service['toggle_cam']()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def cam_select( self, cam ):
		rospy.wait_for_service('ardrone/togglecam')
		try:
			return self.service['cam_select'](cam)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def led_animation( self, animation_type, freq, duration ):
		"""
		animation_type = 
		# 0 : BLINK_GREEN_RED
		# 1 : BLINK_GREEN
		# 2 : BLINK_RED
		# 3 : BLINK_ORANGE
		# 4 : SNAKE_GREEN_RED
		# 5 : FIRE
		# 6 : STANDARD
		# 7 : RED
		# 8 : GREEN
		# 9 : RED_SNAKE
		# 10: BLANK
		# 11: LEFT_GREEN_RIGHT_RED
		# 12: LEFT_RED_RIGHT_GREEN
		# 13: BLINK_STANDARD
		"""

		rospy.wait_for_service('ardrone/setledanimation')
		try:
			return self.service['led_animation'](animation_type, freq, duration)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def imu_calibration( self ):
		rospy.wait_for_service('ardrone/imu_recalib')
		try:
			self.service['imu_calibration']()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def flat_trim( self ):
		rospy.wait_for_service('ardrone/flattrim')
		try:
			self.service['flat_trim']()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def record( self, enable ):
		rospy.wait_for_service('ardrone/setrecord')
		try:
			return self.service['record']( enable )
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def flight_animation( self, animation_type, duration):
		"""
		animation_type = 
		# 0 : ARDRONE_ANIM_PHI_M30_DEG
		# 1 : ARDRONE_ANIM_PHI_30_DEG
		# 2 : ARDRONE_ANIM_THETA_M30_DEG
		# 3 : ARDRONE_ANIM_THETA_30_DEG
		# 4 : ARDRONE_ANIM_THETA_20DEG_YAW_200DEG
		# 5 : ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG
		# 6 : ARDRONE_ANIM_TURNAROUND
		# 7 : ARDRONE_ANIM_TURNAROUND_GODOWN
		# 8 : ARDRONE_ANIM_YAW_SHAKE
		# 9 : ARDRONE_ANIM_YAW_DANCE
		# 10: ARDRONE_ANIM_PHI_DANCE
		# 11: ARDRONE_ANIM_THETA_DANCE
		# 12: ARDRONE_ANIM_VZ_DANCE
		# 13: ARDRONE_ANIM_WAVE
		# 14: ARDRONE_ANIM_PHI_THETA_MIXED
		# 15: ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED
		# 16: ARDRONE_ANIM_FLIP_AHEAD
		# 17: ARDRONE_ANIM_FLIP_BEHIND
		# 18: ARDRONE_ANIM_FLIP_LEFT
		# 19: ARDRONE_ANIM_FLIP_RIGHT
		"""

		rospy.wait_for_service('ardrone/setflightanimation')
		try:
			self.service['flight_animation'](animation_type, 0)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e 

def main():
	pass
	#cmd = Commands()
	#rospy.init_node('Test')
	#cmd = Commands()
	#cmd.TakeOff()
	#cmd.Land()
	#cmd.Reset()

if __name__ == "__main__": main()
