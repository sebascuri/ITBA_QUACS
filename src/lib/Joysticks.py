#!/usr/bin/env python  
# -*- coding: utf-8 -*-
		
class PS3(object):
	"""docstring for PS3"""
	BUTTONS = dict( 
		select 				= 0, 
		L3 					= 1,
		R3 					= 2, 
		start 				= 3, 
		top_arrow 			= 4, 
		right_arrow 		= 5, 
		bottom_arrow 		= 6, 
		left_arrow 			= 7,
		L2 					= 8, 
		R2 					= 9, 
		L1 					= 10, 
		R1 					= 11,
		triangle 			= 12,
		circle 				= 13,
		cross 				= 14,
		square 				= 15,
		)

	AXES = dict(
		L3_LR 				= 0,
		L3_UD 				= 1,
		R3_LR 				= 2,
		R3_UD 				= 3,
		)

	COMMAND = dict(
		R3_UD 	= 'vx',
		R3_LR 	= 'vy',
		L3_UD 	= 'vz',
		L3_LR 	= 'vyaw',
		cross	= 'take_off', 
		circle 	= 'land',
		tringle = 'reset',
		start 	= 'control_on',
		square 	= 'control_off',
		#R1 		= 'chirp_x',
		#R2		= 'chirp_y',
		#L1 		= 'chirp_z',
		#L2		= 'chirp_yaw',
		#select	= 'toggle_cam',
		)

class PS2(object):
	"""docstring for PS2"""
	BUTTONS = dict( 
		triangle = 0,
		circle = 1,
		cross = 2,
		square = 3,
		L1 = 4, 
		R1 = 5,
		L2 = 6, 
		R2 = 7,
		select = 8, 
		start = 9, 
		L3 = 10, 
		R3 = 11,
		analog = 12 
		)

	AXES = dict(
		L3_LR = 0,
		L3_UD = 1,
		R3_LR = 3,
		R3_UD = 4,
		DIR_LR = 6,
		DIR_UD = 7,
		)

	COMMAND = dict(
		R3_UD 	= 'vx',
		R3_LR 	= 'vy',
		L3_UD 	= 'vz',
		L3_LR 	= 'vyaw',
		cross	= 'take_off', 
		circle 	= 'land',
		tringle = 'reset',
		start 	= 'control_on',
		square 	= 'control_off',
		#R1 		= 'chirp_x',
		#R2		= 'chirp_y',
		#L1 		= 'chirp_z',
		#L2		= 'chirp_yaw',
		#select	= 'record',
		)

class Wii(object):
	"""docstring for Wii"""
	BUTTONS = dict( 
		one = 0, 
		two = 1,
		A = 2, 
		B = 3, 
		plus = 4, 
		minus = 5, 
		left = 6, 
		right = 7,
		up = 8, 
		down = 9, 
		home = 10, 
		)

	AXES = dict(
		ax = 0,
		ay = 1, 
		az = 2,
		roll = 3,
		pitch = 4,
		yaw = 5
		)
	
	COMMAND = dict(
		roll = 'vx',
		pitch = 'vy',
		az = 'vz',
		yaw = 'vyaw',
		A = 'take_off',
		B = 'land',
		one = 'control_on',
		two = 'control_off',
		plus = 'chirp_x',
		minus = 'chirp_y',
		left = 'chirp_z',
		right = 'chirp_yaw',
		home = 'reset',
		)