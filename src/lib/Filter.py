#!/usr/bin/env python  
# -*- coding: utf-8 -*-

import numpy as np 
import numpy.matlib as matlib

from collections import deque
import math

from Quaternion import Quaternion
from FilterHelpers import zpk2sos, tf2sos
#import roslib; roslib.load_manifest('ardrone_control')

#import tf

class Digital(object):
	"""docstring for DigitalFilter
		A digital filtered implemented as a Direct Form II parallel algorithm. 
		H(z) = Output / Input = b[i].z^-i / a[i].z^-i 
		ouptut [n] = - a[k] output[n-k] + b[k].input[n-k] 
	"""

	def __init__(self, **kwargs):
		super(Digital, self).__init__()
		self.b = kwargs.get('b', kwargs.get('numerator', [0] )  )
		self.a = kwargs.get('a', kwargs.get('denominator', [0] ) )

		self.input = deque( maxlen = kwargs.get('input_size', len(self.b) ) )

		self.output = deque( maxlen = kwargs.get('output_size', len(self.a)) )

		self.periodic = kwargs.get('periodic', False)

	def set_input(self, new_input):
		self.input.append(new_input)
		self.filter()
		

		return self.get_output()

	def filter(self):
		try:
			new_output = 0 
			for i in range(len(self.b)):
				new_output += self.b[i] * self.input[-1 - i]
			for i in range(len(self.a)):
				new_output -= self.a[i] * self.output[-1 - i]

			if self.periodic:
				new_output = math.atan2( math.sin(new_output), math.cos(new_output) )
				
			self.output.append(new_output)

		except IndexError:
			self.output.append( self.input[-1] )

	def get_output(self):
		try:
			return self.output[-1]
		except IndexError:
			return 0

	def get_output_index( self, n):
		try:
			return self.output[ n ]
		except IndexError:
			return 0 

class SOSDigital(object):
	"""docstring for SOSDigital"""
	def __init__(self, sos_converter):
		super(SOSDigital, self).__init__()
		
		self.filter = []
		for num, den in sos_converter.get_transfer_functions():
			self.filter.append( Digital( b = num, a = den) )

	def set_input(self, new_input):
		self.filter[0].set_input(new_input)
		for i in range(1, len(self.filter)):
			self.filter[i].set_input( self.filter[i-1].get_output() )

		return self.filter[-1].get_output()

	def get_output( self ):
		return self.filter[-1].get_output()

class ZPK(SOSDigital, object):
	"""docstring for ZPK"""
	def __init__(self, zeros, poles, gain, **kwargs):
		super(ZPK, self).__init__( zpk2sos(zeros, poles, gain) )
		
class TF(SOSDigital, object):
	"""docstring for TF"""
	def __init__(self, num, den, **kwargs):
		super(TF, self).__init__( tf2sos(num, den) )
				
class ExtendedKalman(object):
	"""docstring for ExtendedKalmanFilter"""
	def __init__(self, **kwargs):
		super(ExtendedKalman, self).__init__()
		self.Q = np.mat( kwargs.get('Q', [ [0] ] ) ) #process covariance
		self.R = np.mat( kwargs.get('R', [ [0] ] ) ) #measurement covariance
		self.E = np.mat( kwargs.get('E', [ [0] ] ) ) #state covariance

		self.K = np.mat( kwargs.get('K', [ [0] ] ) )#Kalman Gain 

	def predict_covariance(self, J):
		try:
			self.E = J * self.E * J.transpose() + self.Q
		except ValueError:
			self.E = self.Q 

	def correct_covariance(self, H):
		aux = np.dot(self.K , H)
		self.E = np.dot( matlib.eye(aux.shape[0]) - aux  , self.E)

	def correct_state(self, X, Z, Z_expected):
		X += np.dot(self.K , Z - Z_expected)
		return X 

	def correct_gain(self, H):
		self.K = np.dot( np.dot(self.E , H.transpose()) ,
			np.linalg.inv( np.dot(H , np.dot( self.E , H.transpose()) ) + self.R)
			)

	def correct(self, dt, X, Z, Z_expected, H):
		self.correct_gain(H)
		self.correct_covariance(H)
		X = self.correct_state(X, Z, Z_expected)

		return X 

class Kalman(ExtendedKalman, object):
	"""docstring for Kalman"ExtendedKalman """
	def __init__(self,  **kwargs):
		super(Kalman,  self).__init__()
		self.Q = np.mat( kwargs.get('Q', [ [0] ] ) )#process covariance
		self.R = np.mat( kwargs.get('R', [ [0] ] ) )#measurement covariance
		self.E = np.mat( kwargs.get('E', [ [0] ] ) )#state covariance
		
		self.A = np.mat( kwargs.get('A', [ [0] ] ) )#system matrix
		self.B = np.mat( kwargs.get('B', [ [0] ] ) )#input matrix
		self.C = np.mat( kwargs.get('C', [ [0] ] ) )#output matrix

		self.X = np.mat( kwargs.get('X', [ [0] ] ) )#state vector

	def predict_state(self, dt, U):
		Xdot = np.dot(self.A, self.X) + np.dot(self.B, U)
		self.X += Xdot * dt

	def predict(self, dt, U):
		self.predict_state(dt, U)
		self.predict_covariance(self.A)

	def correct(self, dt, Z):
		self.correct_gain(self.C)
		
		self.correct_covariance(self.C)

		self.X = self.correct_state(self.X, Z, np.dot(self.C , self.X) )

class GPS_Odometry(ExtendedKalman, object):
	"""docstring for GPS_Odometry"ExtendedKalman, """
	def __init__(self, **kwargs):
		super(GPS_Odometry, self).__init__(**kwargs)
		self.J = np.mat( kwargs.get('J', matlib.eye( 4 ) ) ) #X = x,y,z,yaw 
		self.X = np.mat( kwargs.get('X', [0,0,0,0]) ).transpose()

		self.E = matlib.zeros( (4,4) ) 

		self.C = np.mat([ 
			[1,0,0,0], 
			[0,1,0,0], 
			[0,0,1,0], 
			])

	def predict_state(self, dt, position, velocity):
		position['x'] += dt * ( velocity['x'] * math.cos(position['yaw'] ) - velocity['y'] * math.sin(position['yaw']) )
		position['y'] += dt * ( velocity['x'] * math.sin(position['yaw'] ) + velocity['y'] * math.cos(position['yaw']) ) 
		position['z'] += dt * ( velocity['z'] )
		
		self.J[0,3] = dt * ( - velocity['x'] * math.sin(position['yaw']) - velocity['y'] * math.cos(position['yaw']) )
		self.J[1,3] = dt * ( + velocity['x'] * math.cos(position['yaw']) + velocity['y'] * math.sin(position['yaw']) )
		
		return position 

	def predict(self, dt, position, velocity):
		position = self.predict_state(dt, position, velocity)

		self.X = np.mat([position['x'],position['y'],position['z'],position['yaw']]).transpose()
		self.predict_covariance(self.J)

		return position 

	def correct(self, position, gps_sensor):
		#measurement = gps_sensor.get_pose()
		measurement = gps_sensor.get_vector()
		Z = np.mat([ measurement[0],measurement[1],measurement[2] ]).transpose()

		self.correct_gain( self.C )
		self.correct_covariance( self.C )
		self.X = self.correct_state( self.X, Z, np.dot(self.C , self.X) )

		position['x'] = self.X.item(0)
		position['y'] = self.X.item(1)
		position['z'] = self.X.item(2)

		return position 
		
class Camera_Odometry(object):
	"""docstring for Camera_Odometry"""
	def __init__(self):
		super(Camera_Odometry, self).__init__()
		
class SO3(object):
	"""docstring for SO3"""
	def __init__(self):
		super(SO3, self).__init__()

	def predict(self, dt, quaternion, gyroscope):
		qdot = quaternion * Quaternion( gyroscope.get_quaternion() )   * 0.5
		quaternion = (quaternion + qdot * dt).normalize()
		return quaternion
		
class Magdwick(SO3, object):
	"""docstring for MagdwickFilter"""
	def __init__(self, **kwargs):
		super(Magdwick, self).__init__()
		#self.Beta = rospy.get_param('Magdwick/Beta', kwargs.get('Beta', 0.1) )

		self.Beta = kwargs.get('Beta', 0.1) 

	def correct(self, dt, quaternion, *sensors):
		if len(sensors) == 0:
			return quaternion
		elif len(sensors) == 1:
			accelerometer = sensors[0].normalized()
			
			f = quaternion.conjugate() * Quaternion( [0, 0, 1.0, 0] ) * quaternion - Quaternion( accelerometer.get_quaternion() )
			
			F = np.mat( f.get_vector() )		
			J = quaternion.z_inv_jacobian()

		elif len(sensors) == 2:
			accelerometer = sensors[0].normalized()
			magnetometer = sensors[1].normalized()

			h = quaternion * Quaternion( magnetometer.get_quaternion() ) * quaternion.conjugate()
			b = [np.linalg.norm([h.x, h.y]), 0, h.z, 0] 
			fg = quaternion.conjugate() * Quaternion( [0, 0, 1.0, 0] ) * quaternion - Quaternion( accelerometer.get_quaternion() )
			fb = quaternion.conjugate() * Quaternion( b ) * quaternion - Quaternion( magnetometer.get_quaternion() )

			F = np.mat(fg.get_vector() +  fb.get_vector())
			Jg = quaternion.z_inv_jacobian()
			Jb = b[0] * quaternion.x_inv_jacobian() + b[2] * quaternion.z_inv_jacobian()
			J = np.bmat( [ [Jg], [Jb] ] )

		return self.correct_error( dt, quaternion, F, J)

	def correct_magnetometer(self, dt, quaternion, magnetometer):
		magnetometer = magnetometer.normalized()

		h = quaternion * Quaternion( magnetometer.get_quaternion() ) * quaternion.conjugate()
		b = [np.linalg.norm([h.x, h.y]), 0, h.z, 0] 
		f = quaternion.conjugate() * Quaternion( b ) * quaternion - Quaternion( magnetometer.get_quaternion() )

		F = np.mat( f.get_vector() )
		J = b[0] * quaternion.x_inv_jacobian() + b[2] * quaternion.z_inv_jacobian()

		return self.correct_error( dt, quaternion, F, J)

 	def correct_accelerometer(self, dt, quaternion, accelerometer):
 		accelerometer = accelerometer.normalized()

		f = quaternion.conjugate() * Quaternion( [0, 0, 1.0, 0] ) * quaternion - Quaternion( accelerometer.get_quaternion() )

		F = np.mat( f.get_vector() )
		J = quaternion.z_inv_jacobian()

		return self.correct_error( dt, quaternion, F, J)

	def correct_error(self, dt, quaternion, F, J):
 		step = np.dot( F, J )
		norm = np.linalg.norm(step)
		if norm != 0:
			step = step/np.linalg.norm(step)
		qdot = Quaternion( - self.Beta * step )

		return (quaternion + qdot * dt).normalize()

class Mahoney(SO3, object):
	"""docstring for MahoneyFilter"""
	def __init__(self, **kwargs):
		super(Mahoney, self).__init__()
		#self.Kp = rospy.get_param('Mahoney/Kp', kwargs.get('Kp', 0.1) )
		#self.Ki = rospy.get_param('Mahoney/Ki', kwargs.get('Ki', 0.1) )
		self.Kp = kwargs.get('Kp', 0.1) 
		self.Ki = kwargs.get('Ki', 0.1) 

		self.e = np.mat([0., 0., 0.])
		self.ei = np.mat([0., 0., 0.])

	def correct(self, dt, quaternion, *sensors):
		if len(sensors) == 0:
			return quaternion
		elif len(sensors) == 1:
			accelerometer = sensors[0].normalized()
			f = quaternion.conjugate() * Quaternion( [0, 0, 1.0, 0] ) * quaternion

			self.e = np.cross( accelerometer.get_vector(), f.get_vector() )

		elif len(sensors) == 2:
			accelerometer = sensors[0].normalized()
			magnetometer = sensors[1].normalized()


			h = quaternion * Quaternion( magnetometer.get_quaternion() ) * quaternion.conjugate()
			b = [np.linalg.norm([h.x, h.y]), 0, h.z, 0] 
			fg = quaternion.conjugate() * Quaternion( [0, 0, 1.0, 0] ) * quaternion
			fb = quaternion.conjugate() * Quaternion( b ) * quaternion

			self.e = np.cross(  accelerometer.get_vector(), fg.get_vector() )
 			self.e += np.cross( magnetometer.get_vector(), fb.get_vector() )


		self.ei += self.e * dt 

 		return self.correct_error( dt, quaternion )

 	def correct_magnetometer(self, dt, quaternion, magnetometer):
		magnetometer = magnetometer.normalized()


		h = quaternion * Quaternion( magnetometer.get_quaternion() ) * quaternion.conjugate()
		b = [np.linalg.norm([h.x, h.y]), 0, h.z, 0] 
		f = quaternion.conjugate() * Quaternion( b ) * quaternion

		self.e = np.cross( magnetometer.get_vector(), f.get_vector() )
		self.ei += self.e * dt 

 		return self.correct_error( dt, quaternion )

 	def correct_accelerometer(self, dt, quaternion, accelerometer):
 		accelerometer = accelerometer.normalized()

		f = quaternion.conjugate() * Quaternion( [0, 0, 1.0, 0] ) * quaternion

		self.e = np.cross(  accelerometer.get_vector(), f.get_vector() )
		self.ei += self.e * dt 

		return self.correct_error( dt, quaternion )

 	def correct_error(self, dt, quaternion):
 		u = 0.5* (self.Kp *self.e + self.Ki*self.ei)
 		qdot = quaternion * Quaternion( np.insert(u, 3, 0) ) 

 		return (quaternion + qdot * dt).normalize()

class ImuCameraPlaneVelocity(object):
	"""docstring for ImuCameraPlaneVelocity"""
	def __init__(self, gamma = 0.5):
		super(ImuCameraPlaneVelocity, self).__init__()
		self.velocity = dict( x = 0.0, y = 0.0)
		self.accelerometer_velocity = dict( x = 0.0, y = 0.0 )
		self.camera_velocity = dict( x = 0.0, y = 0.0 )
		self.gamma = gamma

	def predict( self, unbiased_acceleration, dt ):
		self.accelerometer_velocity['x'] += unbiased_acceleration[0] * dt 
		self.accelerometer_velocity['y'] += unbiased_acceleration[1] * dt 
		self.update_velocity()

	def camera_mesure( self, navdata ):
		self.camera_velocity['x'] = navdata.vx / 1000.0
		self.camera_velocity['y'] = navdata.vy / 1000.0
		self.update_velocity()

	def update_velocity( self ):
		for key in self.velocity.keys():
			self.velocity[key] = self.gamma * self.camera_velocity[key] + (1-self.gamma) * self.accelerometer_velocity[key]
		
	def get_velocity( self ):
		return self.velocity

class ImuRangeVerticalVelocity(	object):
	"""docstring for Imu_Range_Z_Velocity"""
	def __init__(self, gamma = 0.5):
		super(ImuRangeVerticalVelocity, self).__init__()
		self.gamma = gamma
		self.range_velocity = 0.0
		self.accelerometer_velocity = 0.0 
		self.velocity = 0.0

	def predict( self, unbiased_acceleration, dt ):
		self.accelerometer_velocity += unbiased_acceleration[2] * dt 
		self.update_velocity()

	def range_measure( self, range_sensor ):
		self.range_velocity = range_sensor.velocity
		self.update_velocity()

	def update_velocity( self ):
		self.velocity = self.gamma * self.range_velocity + ( 1 - self.gamma ) * self.accelerometer_velocity 		
	
	def get_velocity( self ):
		return dict( z = self.velocity ) 

class VelocityFilter(object):
	"""docstring for VelocityFilter"""
	def __init__(self, gamma_plane = 0.5, gamma_vertical = 0.5):
		super(VelocityFilter, self).__init__()
		self.velocity = dict( x = 0.0, y = 0.0, z = 0.0)
		self.plane_velocity_filter = ImuCameraPlaneVelocity( gamma_plane )
		self.vertical_velocity_filter = ImuRangeVerticalVelocity( gamma_plane )

	def predict(self, unbiased_acceleration, dt):
		self.plane_velocity_filter.predict(unbiased_acceleration, dt )
		self.vertical_velocity_filter.predict(unbiased_acceleration, dt)

	def range_measure( self, range_sensor):
		self.vertical_velocity_filter.range_measure(range_sensor)
		self.update_velocity( )

	def camera_mesure( self, navdata):
		self.plane_velocity_filter.camera_mesure( navdata)
		self.update_velocity( )

	def update_velocity( self ):
		self.velocity.update( self.vertical_velocity_filter.get_velocity() )
		self.velocity.update( self.plane_velocity_filter.get_velocity() )

	def get_velocity( self ):
		return self.velocity
		
def main():
	Digital()
	ZPK( [], [0.2], 1)
	ZPK( [1,1],[0.2,0.3],1 )
	TF( [1,1],[0.2,0.3] )
	ExtendedKalman()
	Kalman()
	GPS_Odometry()
	Camera_Odometry()
	SO3()
	Magdwick()
	Mahoney()


if __name__ == '__main__': main()