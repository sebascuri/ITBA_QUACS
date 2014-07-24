
import numpy as np 
import scipy.signal 
import numpy.linalg 

class zpk2sos(object):
	"""docstring for zpk2sos"""
	def __init__(self, zeros, poles, gain = 1, scale = None):
		super(zpk2sos, self).__init__()
		self.zeros = np.array( zeros ).T 
		self.poles = np.array( poles ).T
		self.gain = gain 

		# Order the poles and zeros in complex conj. pairs and return the number of poles and zeros
		self.zeros.sort()
		self.poles.sort()

		self.lz = self.zeros.size 
		self.lp = self.poles.size 

		self.L = self.lp

		if self.lz > self.lp:
			raise IOError('Too many zeros') 

		# break up conjugate pairs and real poles and zeros
		self.zeros_real, self.zeros_complex = self.break_complex_real( self.zeros )
		self.poles_real, self.poles_complex = self.break_complex_real( self.poles )

		# Order poles according to proximity to unit circle
		self.poles_real = self.order_unit_circle( self.poles_real )
		self.poles_complex = self.order_unit_circle( self.poles_complex )
		self.poles = np.concatenate( (self.poles_complex, self.poles_real) )

		# Order zeros according to proximity to pole pairs 
		self.zeros = self.order_zeros( self.zeros_complex, self.zeros_real, self.poles_complex, self.poles_real )
		# Form SOS matrix
		self.sos  = self.form_sos_matrix()
		if len( self.sos.shape ) == 1:
			self.sos = np.array( [self.sos] )

		#Scale
		self.sos, self.gain = self.scale( self.sos, self.gain, self.L, scale )
		#print self.sos

	def break_complex_real(self, np_array ):
		return np_array[ np.where(np_array.imag == 0) ], np_array[ np.nonzero(np_array.imag) ]

	def order_unit_circle(self, np_array):

		return np.array( [y for (x,y) in   sorted(zip( np.absolute( np_array - np.exp(1j * np.angle(np_array) ) ), np_array ) ) ] )

	def order_zeros(self, z_complex, z_real, p_complex, p_real):
		#Order zeros according to proximity to pole pairs

		new_z = np.array( [] )
				#order the conjugate zero pairs according to proximity to pole pairs
		for i in range(0, z_complex.size/2):
			if p_complex.size is not 0:
				index = np.absolute(z_complex - p_complex[0] ).argmin()
				new_z = np.concatenate( (new_z, np.array([z_complex[  index ]]) ) )
				np.delete(z_complex, [index])

				index = np.absolute(z_complex - p_complex[1] ).argmin()
				new_z = np.concatenate( (new_z, np.array([z_complex[  index ]]) ) )
				z_complex = np.delete(z_complex, [index])

				p_complex = np.delete( p_complex, [0, 1] )

			elif p_real.size is not 0:
				index = np.absolute(z_complex - p_real[0] ).argmin()
				new_z = np.concatenate( (new_z, np.array([z_complex[  index:index+2 ]]) ) )
				z_complex = np.delete(z_complex, [  index, index+1 ])

				p_complex = np.delete( p_complex, [0] )

			else:

				new_z = np.concatenate( (new_z , z_complex) )
				break

		remaining_z = z_real 
		#order remaining real zeros according to proximity to pole pairs too
		for i in range(0, remaining_z.size):
			if p_complex.size is not 0:
				index = np.absolute(remaining_z - p_complex[0] ).argmin()
				new_z = np.concatenate( (new_z, np.array([remaining_z[  index ]]) ) )
				remaining_z = np.delete(remaining_z, [index] )
				p_complex = np.delete( p_complex, [0] )

			elif p_real.size is not 0:
				index = np.absolute(remaining_z - p_real[0] ).argmin()

				new_z = np.concatenate( (new_z, np.array([remaining_z[ index ]]) ) )
				remaining_z = np.delete(remaining_z, index)
				p_real = np.delete( p_real, 0 ) 

			else:
				new_z = np.concatenate( (new_z , remaining_z) )
				break
		
		return new_z  

	def form_sos_matrix(self):
		return self.sos_fun( 0, self.L, self.zeros, self.poles, np.array([]) )

	def sos_fun(self, start, stop, z, p, sos):
		for m in range(start, stop, 2):
			[num, den] = scipy.signal.zpk2tf( z[m:m+2], p[m:m+2], 1 )
			if num.size == 2:
				num = np.concatenate( (np.array([0]), num) )
			elif num.size ==1:
				num = np.concatenate( ( np.array([0]), num, np.array([0]) ) )
			if den.size == 2:
				den = np.concatenate( (den, np.array([0]) ) )
			elif den.size ==1:
				den = np.concatenate( (np.array([0]), den, np.array([0])) )
			if sos.size == 0:
				sos = np.concatenate( (num, den) ) 
			else:
				sos = np.vstack( (np.concatenate( (num, den) ), sos ))
		return sos 

	def scale(self, sos, gain, L, norm_type = None):
		if norm_type == None:
			return sos, gain

		Fnum = 1; 
		Fden = 1; 
		den = sos[0][3:6]
		s = []
		s.append(self.filter_norm( 1, den, norm_type))
		for m in range(1,self.L/2):
			den = sos[m][3:6]
			Fnum = np.convolve( Fnum, sos[m-1][0:3] )
			Fden = np.convolve(Fden, sos[m-1][3:6] )
			Fden2 = np.convolve(Fden,den);

			s.append( self.filter_norm(Fnum, Fden2, norm_type) )

			sos[m-1][0:3] = s[m-1]/s[0] * sos[m-1][0:3]
		sos[-1][0:3] = gain * s[-1]*sos[-1][0:3]
		gain = 1/s[0]

		return sos, gain 

	def filter_norm(self, num, den, norm_type = np.inf ):

		if norm_type == np.inf:
			w,h = scipy.signal.freqz( num, den, whole = True)
			return np.absolute(h).max()
		elif norm_type == 2:
			tout, yout = scipy.signal.dimpulse( (num, den, 1) )
			return scipy.linalg.norm( yout )
		else:
			return 1
	
	def get_transfer_functions( self ):
		for m in range( self.sos.shape[0] ):
			num, den = self.sos[m][0:3], self.sos[m][3:6]
			yield num, den

class tf2sos(zpk2sos):
	"""docstring for TF2SOS"""
	def __init__(self, num, den,  scale = None):
		zeros, poles, gain = scipy.signal.tf2zpk(num, den)

		super(tf2sos, self).__init__(zeros, poles, gain, scale )

def main():
	zeros = [0.5, 0.2, 0.1, -1.1, 0.5+2j, 0.5-2j] 
	poles = [0.3, -0.7, -0.2, -0.7+0.1j, -0.7-0.1j, 0.4]
	gain = 2.  

	zeros = [1,1]
	poles = [0.2, 0.5,]



	zpk = zpk2sos( zeros, poles, gain, np.inf)
	print zpk.sos
	for num, den in zpk.get_transfer_functions():
		print num, den

	num, den = scipy.signal.zpk2tf(zeros, poles, gain)
	tf = tf2sos( num, den )
	for num, den in tf.get_transfer_functions():
		print num, den
	 

	#print zpk.sos 
if __name__ == '__main__':
	main()
