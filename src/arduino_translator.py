#!/usr/bin/env python  
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('ardrone_control');
import rospy

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix


class ArduinoParser(object):
	"""docstring for Arduino"""
	def __init__(self):
		super(ArduinoParser, self).__init__()
		
		self.publisher = dict( 
			gps = rospy.Publisher('ardrone/fix', NavSatFix), 
			mag = rospy.Publisher('ardrone/mag', Vector3Stamped) 
			)

		self.subscriber = dict( 
			gps = rospy.Subscriber('arduino/fix', Vector3Stamped, callback = self.recieve_gps), 
			mag = rospy.Subscriber('arduino/mag', Vector3Stamped, callback = self.recieve_mag)
			)

	def recieve_gps(self, data ):
		msg = NavSatFix()
		msg.latitude = data.vector.x 
		msg.longitude = data.vector.y 
		msg.altitude = data.vector.z

		msg.header.stamp = data.header.stamp
		msg.header.frame_id = data.header.frame_id

		self.publisher['gps'].publish(msg)

	def recieve_mag(self, data ):
		self.publisher['mag'].publish(data)
		
def main():
	rospy.init_node('arduino_translator', anonymous = True)
	arduino_parser = ArduinoParser( )
	rospy.spin()

if __name__ == "__main__": main()