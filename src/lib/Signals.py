#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scipy import signal 
import numpy as np

class SignalResponse(object):
	"""docstring for SignalResponse"""
	def __init__(self, **kwargs):
		super(SignalResponse, self).__init__()
		
		self.t0 = 0
		
		self.tf = kwargs.get('tf', kwargs.get('time', 0) )
		self.dt = kwargs.get('dt', kwargs.get('Ts', kwargs.get('Command_Time', 0) ) )
		self.f = kwargs.get('f', 100)

		self.duty = kwargs.get('f', 0.5)

		self.t = np.arange(self.t0, self.tf, self.dt)

		self.signal = getattr(self, kwargs.get('signal', 'chirp') )()

		self.direction = kwargs.get('direction', 'x')

	def __str__(self):
		return str(self.signal)

	def __len__(self):
		return len(self.signal)

	def __iter__(self):
		for value in self.signal:
			yield value

	def get_signal(self):
		return self.signal

	def command(self):
		return 0.1 * self.signal.pop(0)

	def chirp(self):
		return list( signal.chirp(t = self.t, f0 = 0.01, t1 = self.tf, f1 = self.f) )

	def gausspulse(self):
		return list( signal.gausspulse( t = self.t, fc = self.f) )

	def sawtooth(self):
		return list( signal.sawtooth( t = self.t, width = self.duty) ) 

	def square(self):
		return list( signal.square( t = self.t , duty = self.duty ) )

	def step(self):
		return list( np.ones(np.size(self.t) ) )

def main():
	print SignalResponse( tf = 10, dt = 0.1, f = 10, signal = 'gausspulse' )
	print SignalResponse( tf = 10, dt = 0.1, f = 0.5, signal = 'sawtooth' )
	print SignalResponse( tf = 10, dt = 0.1, f = 0.1, signal = 'square' )
	print SignalResponse( tf = 10, dt = 0.1, f = 10, signal = 'step' )
	signal = SignalResponse( tf = 10, dt = 0.1, f = 10, signal = 'chirp' )
	print signal
	print (signal.signal)

	#for value in signal:
	#	print value 


	#help(scipy.signal.waveforms)

	#help(getattr(scipy.signal, 'chirp'))

if __name__ == '__main__': main()