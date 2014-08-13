#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import rospy 

#from pynmea import nmea 
import pynmea2
import serial 

from sensor_msgs.msg import NavSatFix


COMMAND_TIME = 0.5

class GPSParser(object):
	"""docstring for GPSParser"""
	
	def __init__(self, port_address, baud, **kwargs):
		super(GPSParser, self).__init__()
		
		self.serial = serial.Serial(
			port = port_address, 
			baudrate = baud,
			bytesize = kwargs.get('bytesize', serial.EIGHTBITS ),
			parity = kwargs.get('parity', serial.PARITY_NONE ),
			stopbits = kwargs.get('stopbits', serial.STOPBITS_ONE ),
			timeout = kwargs.get('timeout', None ),
			xonxoff = kwargs.get('xonxoff', False ),
			rtscts = kwargs.get('rtscts', False ),
			writeTimeout = kwargs.get('writeTimeout', None ),
			dsrdtr = kwargs.get('dsrdtr', False ),
			interCharTimeout = kwargs.get('interCharTimeout', None ),
			)

		self.publisher = dict( 
			gps = rospy.Publisher('ardrone/fix', NavSatFix), 
			)

		self.timer = dict( 
			serial = rospy.Timer( rospy.Duration( COMMAND_TIME ), self.read_serial, oneshot=False ),
			)
		
		self.latitude = 0.0
		self.longitude = 0.0
		self.altitude = 0.0

		self.status =  - 1

		self.hdop = 0.0
		self.pdop = 0.0
		self.vdop = 0.0
		
	def read_serial( self, time ):
		while self.serial.inWaiting():
			data = self.serial.readline() 
			try:
				msg = pynmea2.parse(data)

				if msg.sentence_type == 'GGA':
					self.latitude = msg.latitude
					self.longitude = msg.longitude
					self.altitude = msg.altitude

					self.status = int(msg.gps_qual) - 1
					self.talk()

				if msg.sentence_type == 'GSA':
					if self.status == 0:
						self.pdop = float( msg.pdop )
						self.vdop = float( msg.vdop )
					

						try:
							self.hdop = float( msg.hdop )
						except ValueError as e:
							print e, msg.hdop 
							self.hdop = (self.pdop ** 2 - self.vdop**2)**0.5

					else:
						self.pdop = 1000000.0
						self.vdop = 1000000.0
						self.hdop = 1000000.0

			except pynmea2.ParseError as e:
				print data, e

			except pynmea2.ChecksumError as e:
				print data, e

	def write_serial(self, sentence):
		self.serial.write(sentence)

	def close_serial():
		self.serial.close 

	def talk( self ):
		msg = NavSatFix()
		msg.altitude = self.altitude
		msg.longitude = self.longitude
		msg.latitude = self.latitude

		#msg.status = NavSatStatus()
		msg.status.status = self.status
		msg.status.service = 1
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = 'gps'

		
		msg.position_covariance = [
			self.hdop ** 2, 0, 0, 
			0, self.hdop ** 2, 0, 
			0, 0, self.vdop ** 2]
		msg.position_covariance_type = 1 
	


		self.publisher['gps'].publish(msg)


def main():
	rospy.init_node('gps_reciever', anonymous = True)
	port = rospy.get_param('/xbee/port', '/dev/ttyUSB0')
	baud = rospy.get_param('/xbee/baud', 4800)

	GPSParser(port, baud)
	#arduino_parser = ArduinoParser( )
	rospy.spin()

if __name__ == "__main__": main()
