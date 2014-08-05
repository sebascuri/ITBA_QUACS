#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import rospy 

from pynmea import nmea 
#import pynmea
import serial 

from sensor_msgs.msg import NavSatFix


COMMAND_TIME = 0.1

class GPSParser(object):
	"""docstring for GPSParser"""
	Coordinates = dict( 
		N = 1, 
		S = -1, 
		E = 1, 
		W = -1)
	def __init__(self, port_address, baud, **kwargs):
		super(GPSParser, self).__init__()
		
		
		self.parsers = dict( 
			GPGGA = nmea.GPGGA(),
			GPGSA = nmea.GPGSA(),
			# GPRMC = nmea.GPRMC(),
			# GPGSV = nmea.GPGSV(),
			)
		
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
	
	def read_serial( self, time ):
		while self.serial.inWaiting():
			data = self.serial.readline() 

			for key, parser in self.parsers.items():
				if key in data:
					parser.parse(data)
			
		self.talk()

	def write_serial(self, sentence):
		self.serial.write(sentence)

	def close_serial():
		self.serial.close 

	def talk( self ):
		try:
			msg = NavSatFix()
			msg.altitude = float(self.parsers['GPGGA'].antenna_altitude)
			msg.longitude = float(self.parsers['GPGGA'].longitude) * GPSParser.Coordinates[ self.parsers['GPGGA'].lon_direction ]
			msg.latitude = float(self.parsers['GPGGA'].latitude) * GPSParser.Coordinates[ self.parsers['GPGGA'].lat_direction ]

			#msg.status = NavSatStatus()
			msg.status.status = int(self.parsers['GPGGA'].gps_qual) - 1 
			msg.status.service = 1
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = 'gps'

			try:
				msg.position_covariance = [float(self.parsers['GPGSA'].hdop), 0, 0, 0, float(self.parsers['GPGSA'].hdop), 0, 0, 0, float(self.parsers['GPGSA'].vdop)]
				msg.position_covariance_type = 1 
			except ValueError:
				pass


			self.publisher['gps'].publish(msg)
		except AttributeError:
			pass



def main():
	port = rospy.get_param('port_address', '/dev/ttyUSB0')
	baud = rospy.get_param('baud', 4800)

	
	rospy.init_node('gps_reciever', anonymous = True)
	GPSParser(port, baud)
	#arduino_parser = ArduinoParser( )
	rospy.spin()

if __name__ == "__main__": main()
