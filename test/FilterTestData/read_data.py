#! usr/bin/python 
import csv 
import numpy as np

def read_file(filename): 
	L = []
	with open(filename, 'r') as tsv:
		for line in csv.reader(tsv):
			row = []
			for index in range(len(line)):
				row.append( float(line[index]) )
			L.append(row)
	return L

def main():
	gyroscope = np.mat(read_file('Gyroscope.txt'))
	accelerometer = np.mat(read_file('Accelerometer.txt'))
	magnetometer = np.mat(read_file('Magnetometer.txt'))
	time = np.mat(read_file('Time.txt'))

	quaternion_madgwick_imu = np.roll( np.mat( read_file('quaternion_madgwick_imu.txt') ), -1)
	
	quaternion_madgwick_marg = np.roll( np.mat( read_file('quaternion_madgwick_marg.txt') ), -1 )
	quaternion_mahoney_imu = np.roll( np.mat( read_file('quaternion_mahoney_imu.txt') ), -1 )
	quaternion_mahoney_marg = np.roll( np.mat( read_file('quaternion_mahoney_marg.txt') ), -1 )

if __name__ == '__main__':
	main()

