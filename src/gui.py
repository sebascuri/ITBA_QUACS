#!/usr/bin/env python  
# -*- coding: utf-8 -*-

import rospy 
import roslaunch 

from ardrone_commander import JoystickController
from lib.Joysticks import PS2, PS3, Wii
from lib.Quadrotor import ArDroneStates 
from lib.Quaternion import euler_from_quaternion

from lib.Signals import SignalResponse
from ardrone_control.srv import OpenLoopCommand 
from ardrone_control.msg import QuadrotorPose
from ardrone_autonomy.msg import Navdata 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import QuaternionStamped as QuaternionMsg

from sensor_msgs.msg import CompressedImage, Image 
from nav_msgs.msg import Odometry
import dynamic_reconfigure.client


from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import os 
import sys
from collections import deque, OrderedDict
import imp
from functools import partial
import numpy as np 

import inspect

from PyQt4 import QtGui, QtCore

import math
image_format_table = {'rgb8': QtGui.QImage.Format_RGB888, 'mono8': QtGui.QImage.Format_Mono}
COMMAND_TIME = 0.01 #s
REFRESH_GUI_TIME = 500 #ms
ERROR_WAY_POINT = 0.5

class MplPlot(object):
	"""docstring for MplPlot"""
	def __init__(self ):
		super(MplPlot, self).__init__()
		self.x = deque( maxlen = 500 )
		self.y = deque( maxlen = 500 )

	def append( self , x, y):
		self.x.append( x )
		self.y.append( y )
		
class ArDronePlotCanvas(FigureCanvas):
	"""docstring for MyPlot"""
	COLOR_LIST = [
		'r', 'g', 'b', 'y', 'c', 'm', 'k',
		'r--', 'g--', 'b--', 'y--', 'c--', 'm--', 'k--'] 
	
	def __init__(self, *args, **kwargs):
		self.fig = Figure( figsize=(kwargs.get('width'), kwargs.get('height')), dpi=kwargs.get('dpi') )
		self.axes = self.fig.add_subplot(111)
		self.axes.hold(True)
		self.axes.set_xlabel('time [s]')
		self.fig.tight_layout()
		
		attribute_list = ['position', 'velocity', 'setPoint', 'cmdVel', 'quaternion']

		self.checkBox = dict()
		self.plots = dict()
		self.plotHandles = dict()
		self.yLabels = dict()
		
		self.activePlots = []
		for attribute in attribute_list:
			self.checkBox[attribute] = dict()
			self.plots[attribute] = dict()
			self.plotHandles[attribute] = dict()
			self.yLabels[attribute] = dict()
			#self.activePlots[attribute] = dict()

		variables = ['x', 'y', 'z', 'yaw']
		for dictionary in self.plots.values():
			for variable in variables:
				dictionary[variable] = 0.0

		for attribute, yLabel in self.yLabels.items():
			if attribute == 'quaternion':	
				yLabel['x'] = yLabel['y'] = yLabel['z'] = yLabel['w'] = ''
			else:
				for variable in variables:
					yLabel[variable] = '[{0}{1}]'.format( 'm' if variable != 'yaw' else 'rad' , 
						'' if (attribute == 'position' or attribute == 'setPoint') else '/s' )

		del self.plots['quaternion']['yaw']
		self.plots['quaternion']['w'] = 1.0


		self.reset()



		FigureCanvas.__init__(self, self.fig)

		FigureCanvas.setSizePolicy(self, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
		FigureCanvas.updateGeometry(self)
	
	def selectVariablesDialog( self ):
		widget = QtGui.QDialog()
		layout = QtGui.QGridLayout()
		self.reset()
		i = 0
		keys = ['x', 'y', 'z', 'yaw']
		for plot_names, dictionaries in self.plots.items():
			self.checkBox[plot_names] = dict()
			for key in keys:
				if plot_names == 'quaternion' and key == 'yaw':
					key = 'w'

				self.checkBox[plot_names][key] = QtGui.QCheckBox( '{0} {1}'.format(key, plot_names), self)
				layout.addWidget( self.checkBox[plot_names][key], i/4, i%4 )
				i += 1


		self.button = QtGui.QDialogButtonBox( 
			QtGui.QDialogButtonBox.Ok|QtGui.QDialogButtonBox.Cancel, 
			QtCore.Qt.Horizontal )

		widget.connect( self.button.button( QtGui.QDialogButtonBox.Ok ), QtCore.SIGNAL('clicked()'), self.onOk )
		widget.connect( self.button.button( QtGui.QDialogButtonBox.Cancel ), QtCore.SIGNAL('clicked()'), self.onCancel )
		widget.connect( self.button, QtCore.SIGNAL("rejected()"), widget, QtCore.SLOT("reject()"))
		widget.connect( self.button, QtCore.SIGNAL("accepted()"), widget, QtCore.SLOT("accept()"))

		layout.addWidget(self.button, i, 3)


		widget.setLayout(layout)
		widget.setWindowTitle('Select Variables Dialog')
		widget.show()
		widget.exec_()	

	def onOk(self):
		for attribute, checkBoxGroup in self.checkBox.items():
			for variable, checkBox in checkBoxGroup.items():
				if checkBox.isChecked():
					self.activePlots.append( (attribute, variable) )

		#self.activePlots = [name for name in self.plotNames if self.checkBox[name].isChecked()]
		self.createPlotHandles()

	def onCancel(self):
		self.reset()
		self.createPlotHandles()

	def reset( self ):
		self.plotHandles = dict()
		
		self.activePlots = list()

		for dictionary in self.plots.values():
			for key in dictionary.keys():
				dictionary[key] = MplPlot()
			#self.plots[key] = MplPlot()

	def createPlotHandles( self ):
		self.axes.clear() 
		#self.axes.legend_ = None
		i = 0
		yLabel = set()
		for activePlot in self.activePlots:
			self.plotHandles[activePlot], = self.axes.plot( [], [], self.COLOR_LIST[i], linewidth=2, label=
				'{0} {1} {2}'.format(activePlot[0], activePlot[1], self.yLabels[activePlot[0]][activePlot[1]] ) ) 
			yLabel.add( self.yLabels[activePlot[0]][activePlot[1]] )
			i+=1

		self.axes.set_xlabel('time [s]')
		if len(yLabel) == 1:
			self.axes.set_ylabel(yLabel.pop())

		self.axes.legend( self.plotHandles.values(), [plot.get_label() for plot in self.plotHandles.values()], loc='best', shadow=True)
		self.fig.tight_layout()
		self.draw()

	def updateFigure(self):

		for activePlot, plotHandle in self.plotHandles.items():
			plotHandle.set_data( self.plots[activePlot[0]][activePlot[1]].x , self.plots[activePlot[0]][activePlot[1]].y)
		
		self.axes.relim()
		self.axes.autoscale_view()
		self.fig.tight_layout()

		self.draw()
		self.flush_events()

class CameraCanvas(QtGui.QLabel):
	"""docstring for CameraCanvas"""
	def __init__(self):
		super(CameraCanvas, self).__init__()

		self.subscriber = rospy.Subscriber('ardrone/image_raw', Image, callback = self.recieveImage)

		self.pixmap = QtGui.QPixmap.fromImage( 
			QtGui.QImage( os.path.dirname(os.path.abspath(__file__))+'/figs/freeflight.png')
			)
		self.setScaledContents(True)

	def recieveImage(self, image):
		self.pixmap = QtGui.QPixmap.fromImage( 
			QtGui.QImage(image.data, image.width, image.height, image.step , image_format_table[image.encoding] )
			)
		
	def updateCamera(self):
		#self.setPixmap( self.pixmap.scaled( self.width(), self.height(), QtCore.Qt.KeepAspectRatioByExpanding) )
		self.setPixmap( self.pixmap.scaled( self.width(), self.height(), QtCore.Qt.KeepAspectRatio) )

class ControllerSetupWizard(QtGui.QWizard):
	"""docstring for ControllerSetupWizard"""
	def __init__(self):
		super(ControllerSetupWizard, self).__init__()
		self.ok = False
		self.pages = dict()
		self.gains = dict()
		self.gain_values = dict()

		self.lineEdit = dict()
		self.lineEditLabel = dict()

		self.controller = dict()

		self.pageList = [ ' ',
			'Intro', 'ControllerType', 'PID', 'ZPK', 'TF', 
			'Conclusion'
		]


		self.controller_type = "PID"
		self.direction = 'x'
		self.digital = True

		self.createIntroPage( self.pageList.index('Intro') )
		self.createControllerTypePage( self.pageList.index('ControllerType') )
		self.createPIDGainsPage(self.pageList.index('PID'))
		self.createZPKPage(self.pageList.index('ZPK'))
		self.createTFPage(self.pageList.index('TF'))


		self.createConclusionPage( self.pageList.index('Conclusion') )

		QtCore.QObject.connect( self.button(self.FinishButton), QtCore.SIGNAL('clicked()'), self.onFinish )
		QtCore.QObject.connect( self.button(self.CancelButton), QtCore.SIGNAL('clicked()'), self.onCancel )

		self.setWindowTitle("Controller Wizard")
		self.show()
		self.exec_()

	def createIntroPage( self , page_num):
		self.pages['intro'] = QtGui.QWizardPage()
		self.pages['intro'].setTitle("Introduction")

		label = QtGui.QLabel("This wizard will help you configure the Ar.Drone Controller")
		label.setWordWrap(True)

		layout = QtGui.QVBoxLayout()
		layout.addWidget(label)
		self.pages['intro'].setLayout(layout)

		self.setPage( page_num, self.pages['intro'] )

	def createControllerTypePage( self, page_num ):
		self.pages['controllerType'] = QtGui.QWizardPage()
		self.pages['controllerType'].setTitle("Controller Type")

		comboType = QtGui.QComboBox(self); 
		comboType.addItems( [ "PID", "Zero-Pole Gain", "Transfer Function"] ); 
		comboType.activated[str].connect( self.onControllerSelect )

		comboDir = QtGui.QComboBox(self); 
		comboDir.addItems( [ "x", "y", "z ", "yaw"] ); 
		comboDir.activated[str].connect( self.onDirectionSelect )

		comboInput = QtGui.QComboBox(self); 
		comboInput.addItems( [ "Digital", "Continuous" ] ); 
		comboInput.activated[str].connect( self.onDigitalSelect )

		self.TsBox = QtGui.QDoubleSpinBox()
		self.TsBox.setRange(0.01, 1); 
		self.TsBox.setSingleStep(0.01)
		TsLabel = QtGui.QLabel("Insert Sampling Time [s]")
		
		TsBoxLayout = QtGui.QHBoxLayout()
		TsBoxLayout.addWidget( TsLabel )
		TsBoxLayout.addWidget( self.TsBox )

		layout = QtGui.QVBoxLayout()
		layout.addWidget( comboType )
		layout.addWidget( comboDir )
		layout.addWidget( comboInput )
		layout.addLayout( TsBoxLayout )

		self.pages['controllerType'].setLayout(layout)

		self.setPage(page_num, self.pages['controllerType'] )

	def createPIDGainsPage( self, page_num   ):
		self.pages['pidGains'] = QtGui.QWizardPage()
		self.pages['pidGains'].setTitle( " PID Gains")

		self.gains = dict()
		self.gains['proportional'] = QtGui.QSlider(QtCore.Qt.Horizontal, self)
		self.gains['integral'] = QtGui.QSlider(QtCore.Qt.Horizontal, self)
		self.gains['derivative'] = QtGui.QSlider(QtCore.Qt.Horizontal, self)

		self.gain_values = dict()
		for key, slider in self.gains.items():
			self.gain_values[key] = QtGui.QLabel( str(0.01*float(slider.value())) )
		

		layout = QtGui.QGridLayout()
		i = 0
		for key, slider in self.gains.items():
			slider.setRange(0, 200)
			label = self.gain_values[key]
			layout.addWidget(QtGui.QLabel(key), i, 0)
			layout.addWidget(slider,i, 1)
			slider.valueChanged[int].connect( self.changeGain )

			layout.addWidget(self.gain_values[key], i, 2)
			i += 1

		self.pages['pidGains' ].setLayout(layout)

		self.setPage(page_num, self.pages['pidGains' ] )

	def createZPKPage( self, page_num):
		self.pages['zpkPage'] = QtGui.QWizardPage()
		self.pages['zpkPage'].setTitle(" Controller Zero-Pole Gain Configuration")
		
		self.lineEdit['zpk'] = dict()
		self.lineEdit['zpk']['gain'] = QtGui.QLineEdit("2.25")
		self.lineEditLabel['gain'] = QtGui.QLabel("Insert <b>Gain</b>")

		self.lineEdit['zpk']['zeros'] = QtGui.QLineEdit("1.15, -0.83")
		self.lineEditLabel['zeros'] = QtGui.QLabel("Insert <b>Zeros</b> separated by commas")

		self.lineEdit['zpk']['poles'] = QtGui.QLineEdit("0.331, -0.14, 0.17")
		self.lineEditLabel['poles'] = QtGui.QLabel("Insert <b>Poles</b> separated by commas")


		layout = QtGui.QGridLayout()
		i = 0
		for attribute, lineEdit in self.lineEdit['zpk'].items():
			layout.addWidget( self.lineEditLabel[attribute], i, 0 )
			layout.addWidget( lineEdit, i, 1 )
			i+=1

		self.pages['zpkPage'].setLayout(layout)

		self.setPage( page_num, self.pages['zpkPage'] )

	def createTFPage( self, page_num ):
		self.pages['tfPage'] = QtGui.QWizardPage()
		self.pages['tfPage'].setTitle("Transfer Function Controller Setup")

		self.lineEdit['tf'] = dict()
		self.lineEdit['tf']['numerator'] = QtGui.QLineEdit("1.15, -0.83")
		self.lineEditLabel['numerator'] = QtGui.QLabel("Insert <b>numerator</b> coefficients separated by commas")

		self.lineEdit['tf']['denominator'] = QtGui.QLineEdit("0.331, -0.14, 0.17")
		self.lineEditLabel['denominator'] = QtGui.QLabel("Insert <b>denominator</b> coefficients separated by commas")


		layout = QtGui.QVBoxLayout()
		for attribute, lineEdit in self.lineEdit['tf'].items():
			layout.addWidget( self.lineEditLabel[attribute] )
			layout.addWidget( lineEdit )

		self.pages['tfPage'].setLayout(layout)

		self.setPage( page_num, self.pages['tfPage'] )

	def createConclusionPage(self, page_num):
		self.pages['conclusion'] = QtGui.QWizardPage()
		self.pages['conclusion'].setTitle("Conclusion")

		label = QtGui.QLabel("You have succesfully set up a new controller \nLet's Try How it Works!")
		label.setWordWrap(True)

		layout = QtGui.QVBoxLayout()
		layout.addWidget(label)
		self.pages['conclusion'].setLayout(layout)

		self.setPage( page_num, self.pages['conclusion'] )

	def onControllerSelect(self, controller):
		self.controller_type = str(controller )

	def onDirectionSelect(self, direction):
		self.direction = str(direction)

	def onDigitalSelect( self, digital):
		self.digital = ('Digital' == str(digital))
	
	def changeGain(self):
		for key, slider in self.gains.items():
			self.gain_values[key].setText( str(0.01*float(slider.value())) ) 

	def nextId( self ):
		current = self.currentId() 
		if current == self.pageList.index('ControllerType'):
			if 'PID' in self.controller_type:
				return self.pageList.index('PID')
			elif 'Transfer Function' in self.controller_type:
				return self.pageList.index('TF')
			else:
				return self.pageList.index('ZPK')
		elif current == self.pageList.index('PID') or current == self.pageList.index('ZPK') or current == self.pageList.index('TF'):
			return self.pageList.index('Conclusion')
		elif current == self.pageList.index('Conclusion'):
			return -1
		else:
			return current + 1 

	def getControllers( self ):
		parameter = dict()
		if 'PID' in self.controller_type:
			for gain, slider in self.gains.items():
				parameter[gain] = 0.01*float(slider.value())
		elif 'Zero-Pole Gain' == self.controller_type:
			for attribute, line_edit in self.lineEdit['zpk'].items():
				parameter[attribute] = str( line_edit.text().simplified() )
		elif 'Transfer Function' == self.controller_type:
			for attribute, line_edit in self.lineEdit['tf'].items():
				parameter[attribute] = str( line_edit.text().simplified() )

		return parameter 

	def getValues( self ):
		return self.controller_type, self.direction, self.digital, self.TsBox.value(), self.getControllers(), self.ok

	def onFinish( self ):
		self.ok = True

	def onCancel( self ):
		self.ok = False

class InputSetPointDialog(QtGui.QDialog):
	"""docstring for InputSetPoint"""
	def __init__(self ):
		super(InputSetPointDialog, self).__init__()
		self.ok = False
		self.coordinates = ['x', 'y', 'z', 'yaw']
		self.labels = dict()
		self.box = dict()

		#QtGui.QDoubleSpinBox()
		
		layout = QtGui.QGridLayout()
		layout.addWidget( QtGui.QLabel('Set Point Type'), 0, 0 )
		combo = QtGui.QComboBox (); combo.addItems(['Position', 'Velocity'])
		combo.activated[str].connect( self.onSetPointSelect )
		self.set_point_type = 'Position'

		layout.addWidget( combo, 0, 1 )
		i = 1
		for coordinate in self.coordinates:
			self.labels[coordinate] = QtGui.QLabel( '{0} Set Point [{1}]:'.format(
				coordinate, 'deg' if coordinate == 'yaw' else 'm') 
			)

			self.box[coordinate] = QtGui.QDoubleSpinBox()
			self.box[coordinate].setRange(-50, 50); self.box[coordinate].setSingleStep(0.5)

			layout.addWidget(self.labels[coordinate], i, 0)
			layout.addWidget(self.box[coordinate], i, 1)
			i += 1
		self.box['yaw'].setRange(-180, 180);
		self.button = QtGui.QDialogButtonBox( 
			QtGui.QDialogButtonBox.Ok|QtGui.QDialogButtonBox.Cancel, 
			QtCore.Qt.Horizontal )

		layout.addWidget(self.button, i, 0)


		self.connect( self.button.button( QtGui.QDialogButtonBox.Ok ), QtCore.SIGNAL('clicked()'), self.onOk )
		self.connect( self.button.button( QtGui.QDialogButtonBox.Cancel ), QtCore.SIGNAL('clicked()'), self.onCancel )
		self.connect(self.button, QtCore.SIGNAL("rejected()"), self, QtCore.SLOT("reject()"))
		self.connect(self.button, QtCore.SIGNAL("accepted()"), self, QtCore.SLOT("accept()"))


		self.setLayout(layout)

		self.setWindowTitle("Input Set Point")
		self.show()
		self.exec_()

	def onSetPointSelect(self, string):
		self.set_point_type = string

	def onOk( self ):
		self.ok = True

	def onCancel( self ):
		self.ok = False

	def getValues( self ):
		return self.set_point_type, {coordinate:self.box[coordinate].value() for coordinate in self.coordinates}, self.ok 
		
class SignalSetupDialog(QtGui.QDialog):
	"""docstring for SignalSetupDialog"""
	def __init__(self):
		super(SignalSetupDialog, self).__init__()
		self.ok = False
		signal_list = ["chirp", "gausspulse", "sawtooth", "square", "step"]
		coordinate_list = ["x", "y", "z", "yaw"]
		input_list = ['signal', 'coordinate', 'duration', 'period', 'frequency']

		self.dutyCycle = False

		self.labels = dict()
		self.values = dict()

		self.labels['signal'] = QtGui.QLabel("Signal Type")
		self.values['signal'] = QtGui.QComboBox(); self.values['signal'].addItems(signal_list)

		self.labels['coordinate'] = QtGui.QLabel("Coordinate List")
		self.values['coordinate'] = QtGui.QComboBox(); self.values['coordinate'].addItems(coordinate_list)

		self.connect(self.values['signal'], QtCore.SIGNAL('activated(QString)'), self.chooseSignal)

		self.labels['duration'] = QtGui.QLabel("Duration [sec]")
		self.values['duration'] = QtGui.QSpinBox(); self.values['duration'].setRange(0, 60)

		self.labels['period'] = QtGui.QLabel("Sampling Interval [sec]")
		self.values['period'] = QtGui.QDoubleSpinBox(); self.values['period'].setRange(0, 1); self.values['period'].setSingleStep(0.05);

		self.labels['frequency'] = QtGui.QLabel("Signal Frequency [Hz]")
		self.values['frequency'] = QtGui.QDoubleSpinBox(); self.values['frequency'].setRange(0, 100); self.values['frequency'].setSingleStep(1);

		self.button = QtGui.QDialogButtonBox( 
			QtGui.QDialogButtonBox.Ok|QtGui.QDialogButtonBox.Cancel, 
			QtCore.Qt.Horizontal )

		self.connect( self.button.button( QtGui.QDialogButtonBox.Ok ), QtCore.SIGNAL('clicked()'), self.onOk )
		self.connect( self.button.button( QtGui.QDialogButtonBox.Cancel ), QtCore.SIGNAL('clicked()'), self.onCancel )
		self.connect(self.button, QtCore.SIGNAL("rejected()"), self, QtCore.SLOT("reject()"))
		self.connect(self.button, QtCore.SIGNAL("accepted()"), self, QtCore.SLOT("accept()"))

		layout = QtGui.QGridLayout()
		i = 0
		for input_ in input_list:
			layout.addWidget(self.labels[input_], i, 0)
			layout.addWidget(self.values[input_], i, 1)
			i+=1

		layout.addWidget(self.button, len(input_list)+1, 0)
		self.setLayout(layout)

		self.setWindowTitle("Open Loop Signals Dialog")
		self.show()
		self.exec_()

	def chooseSignal(self, signal):
		if signal == "step":
			self.dutyCycle = False
			self.labels['frequency'].setEnabled(False)
			self.values['frequency'].setEnabled(False)
		else:
			self.labels['frequency'].setEnabled(True)
			self.values['frequency'].setEnabled(True)

			if signal == "gausspulse" or signal == "chirp":
				self.dutyCycle = False
				self.labels['frequency'].setText("Signal Frequency [Hz]")
				self.values['frequency'] = QtGui.QDoubleSpinBox(); self.values['frequency'].setRange(0, 100); self.values['frequency'].setSingleStep(1);

			elif signal == "square" or signal == "sawtooth":
				self.dutyCycle = False
				self.labels['frequency'].setText("Duty Cycle  [%]")
				self.values['frequency'] = QtGui.QDoubleSpinBox(); self.values['frequency'].setRange(0, 100); self.values['frequency'].setSingleStep(1);

	def onOk( self ):
		self.ok = True

	def onCancel( self ):
		self.ok = False

	def _getData( self ):
		data = OpenLoopCommand()
		data.time = self.values['duration'].value()
		data.dt = self.values['period'].value() 
		data.f = self.values['frequency'].value()/100.0 if self.dutyCycle else self.values['frequency'].value()
		data.signal = str( self.values['signal'].currentText() )
		data.direction = str( self.values['coordinate'].currentText() ) 

		return data

	def getData( self ):
		return self._getData(), self.ok 

class ArduinoSetupDialog(QtGui.QDialog):
	"""docstring for SignalSetupDialog"""
	def __init__(self):
		super(ArduinoSetupDialog, self).__init__()
		self.ok = False
		self.baudList = [110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000, 57600, 115200, 
		128000, 153600, 230400, 256000, 460800, 921600 ]

		input_list = ['baud', 'port']
		self.labels = dict()
		self.values = dict()

		self.labels['baud'] = QtGui.QLabel("Baud Rate")
		self.values['baud'] = QtGui.QComboBox(); self.values['baud'].addItems([ str(a) for a in self.baudList ])
		self.values['baud'].setCurrentIndex( self.baudList.index(4800) )

		self.labels['port'] = QtGui.QLabel("Select Port ")
		self.values['port'] = QtGui.QLineEdit("/dev/ttyACM1")

		self.button = QtGui.QDialogButtonBox( 
			QtGui.QDialogButtonBox.Ok|QtGui.QDialogButtonBox.Cancel, 
			QtCore.Qt.Horizontal )

		self.connect( self.button.button( QtGui.QDialogButtonBox.Ok ), QtCore.SIGNAL('clicked()'), self.onOk )
		self.connect( self.button.button( QtGui.QDialogButtonBox.Cancel ), QtCore.SIGNAL('clicked()'), self.onCancel )
		self.connect(self.button, QtCore.SIGNAL("rejected()"), self, QtCore.SLOT("reject()"))
		self.connect(self.button, QtCore.SIGNAL("accepted()"), self, QtCore.SLOT("accept()"))

		layout = QtGui.QGridLayout()
		i = 0
		for input_ in input_list:
			layout.addWidget(self.labels[input_], i, 0)
			layout.addWidget(self.values[input_], i, 1)
			i+=1

		layout.addWidget(self.button, 3, 0)
		self.setLayout(layout)

		self.setWindowTitle("Open Loop Signals Dialog")
		self.show()
		self.exec_()

	def onOk( self ):
		self.ok = True

	def onCancel( self ):
		self.ok = False

	def getData( self ):
		return int( self.baudList[ self.values['baud'].currentIndex() ]), str(self.values['port'].text().simplified()), self.ok 

class XbeeSetupDialog(QtGui.QDialog):
	"""docstring for SignalSetupDialog"""
	def __init__(self):
		super(XbeeSetupDialog, self).__init__()
		self.ok = False
		self.baudList = [110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000, 57600, 115200, 
		128000, 153600, 230400, 256000, 460800, 921600 ]

		input_list = ['baud', 'port']
		self.labels = dict()
		self.values = dict()

		self.labels['baud'] = QtGui.QLabel("Baud Rate")
		self.values['baud'] = QtGui.QComboBox(); self.values['baud'].addItems([ str(a) for a in self.baudList ])
		self.values['baud'].setCurrentIndex( self.baudList.index(4800) )

		self.labels['port'] = QtGui.QLabel("Select Port ")
		self.values['port'] = QtGui.QLineEdit("/dev/ttyUSB0")

		self.button = QtGui.QDialogButtonBox( 
			QtGui.QDialogButtonBox.Ok|QtGui.QDialogButtonBox.Cancel, 
			QtCore.Qt.Horizontal )

		self.connect( self.button.button( QtGui.QDialogButtonBox.Ok ), QtCore.SIGNAL('clicked()'), self.onOk )
		self.connect( self.button.button( QtGui.QDialogButtonBox.Cancel ), QtCore.SIGNAL('clicked()'), self.onCancel )
		self.connect(self.button, QtCore.SIGNAL("rejected()"), self, QtCore.SLOT("reject()"))
		self.connect(self.button, QtCore.SIGNAL("accepted()"), self, QtCore.SLOT("accept()"))

		layout = QtGui.QGridLayout()
		i = 0
		for input_ in input_list:
			layout.addWidget(self.labels[input_], i, 0)
			layout.addWidget(self.values[input_], i, 1)
			i+=1

		layout.addWidget(self.button, 3, 0)
		self.setLayout(layout)

		self.setWindowTitle("Open Loop Signals Dialog")
		self.show()
		self.exec_()

	def onOk( self ):
		self.ok = True

	def onCancel( self ):
		self.ok = False

	def getData( self ):
		return int( self.baudList[ self.values['baud'].currentIndex() ]), str(self.values['port'].text().simplified()), self.ok 

class InitialConditionDialog(QtGui.QDialog):
	"""docstring for InitialConditionDialog"""
	def __init__(self):
		super(InitialConditionDialog, self).__init__()
		self.ok = False
		self.position_list = ['x', 'y', 'z']
		self.angles_list = ['yaw']
		self.labels = dict()
		self.box = dict()
		self.text = dict()
		tabWidget = QtGui.QTabWidget()

		tabWidget.addTab(self.NameTab(), "Drone Name")
		tabWidget.addTab(self.initialCoordinateTab('position'), "Initial Position")
		#tabWidget.addTab(self.initialCoordinateTab('velocity'), "Initial Velocity")

		tabWidget.addTab(self.onBoardSensors(), "On Board Sensors")


		self.button = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel)

		self.connect( self.button.button( QtGui.QDialogButtonBox.Ok ), QtCore.SIGNAL('clicked()'), self.onOk )
		self.connect( self.button.button( QtGui.QDialogButtonBox.Cancel ), QtCore.SIGNAL('clicked()'), self.onCancel )
		self.connect( self.button, QtCore.SIGNAL("rejected()"), self, QtCore.SLOT("reject()"))
		self.connect( self.button, QtCore.SIGNAL("accepted()"), self, QtCore.SLOT("accept()"))

		mainLayout = QtGui.QVBoxLayout()
		mainLayout.addWidget(tabWidget)
		mainLayout.addWidget(self.button)
		self.setLayout(mainLayout)

		self.resize(400, 70)

		self.setWindowTitle("Initial Conditions Dialog")
		self.show()
		self.exec_()

	def NameTab(self):
		widget =  QtGui.QWidget()
		layout = QtGui.QHBoxLayout()
		self.labels['name'] = QtGui.QLabel( 'Input Drone Name' )
		self.text['name'] = QtGui.QLineEdit("/local")

		layout.addWidget( self.labels['name'] )
		layout.addWidget( self.text['name'] )

		widget.setLayout(layout)
		return widget
	
	def initialCoordinateTab(self, attribute):
		widget =  QtGui.QWidget()
		layout = QtGui.QGridLayout()

		i = 0
		self.labels[attribute] = dict()
		self.box[attribute] = dict()
		for coordinate in self.position_list:
			self.labels[attribute][coordinate] = QtGui.QLabel( '{0} {1} [m]:'.format(coordinate, attribute) )
			self.box[attribute][coordinate] = QtGui.QDoubleSpinBox()
			self.box[attribute][coordinate].setRange(-100, 100)

			layout.addWidget(self.labels[attribute][coordinate], i, 0)
			layout.addWidget(self.box[attribute][coordinate], i, 1)
			i += 1

		for coordinate in self.angles_list:
			self.labels[attribute][coordinate] = QtGui.QLabel( '{0} {1} [deg]:'.format(coordinate, attribute) )
			self.box[attribute][coordinate] = QtGui.QDoubleSpinBox()
			self.box[attribute][coordinate].setRange(-180, 180)

			layout.addWidget(self.labels[attribute][coordinate], i, 0)
			layout.addWidget(self.box[attribute][coordinate], i, 1)
			i += 1

		widget.setLayout(layout)
		return widget

	def onBoardSensors(self):
		widget =  QtGui.QWidget()
		layout = QtGui.QVBoxLayout()
		sensor_list = ['Accelerometer', 'Magnetometer', 'Gyroscope', 'Range', 'GPS', 'Camera']
		self.sensors = dict()
		for sensor in sensor_list:
			self.sensors[sensor] = QtGui.QCheckBox(sensor, self)
			layout.addWidget( self.sensors[sensor] )
		self.sensors['Accelerometer'].setCheckState(QtCore.Qt.Checked)
		self.sensors['Gyroscope'].setCheckState(QtCore.Qt.Checked)
		self.sensors['Magnetometer'].setCheckState(QtCore.Qt.Checked)
		self.sensors['GPS'].setCheckState(QtCore.Qt.Checked)
		self.sensors['Range'].setCheckState(QtCore.Qt.Checked)

		widget.setLayout(layout)
		return widget

	def onOk( self ):
		self.ok = True

	def onCancel( self ):
		self.ok = False

	def getName(self):
		return str( self.text['name'].text().simplified() )

	def getCoordinates(self, attribute):
		d = dict()
		for position in self.position_list:
			d[position] = self.box[attribute][position].value()
		for angle in self.angles_list:
			d[angle] = self.box[attribute][angle].value() * math.pi / 180.0

		return d

	def getSensors(self):
		#return [sensor for sensor in self.sensors.keys() if bool(self.sensors[sensor].checkState()) ]
		return {sensor_name: bool(sensor_box.checkState()) for (sensor_name, sensor_box) in self.sensors.items() }

	def getData(self):
		return self.getName(), self.getCoordinates('position'), self.getSensors(), self.ok

class WayPoints(object):
	"""docstring for WayPoints"""
	def __init__(self, x, y, z, *yaw):
		super(WayPoints, self).__init__()

		self.properties = dict()
		
		self.x = x
		self.y = y 
		self.z = z
		if len(yaw) > 0:
			self.yaw = yaw[0]
		else:
			self.yaw = [0 for a in x]

	def __len__(self):
		return len(self.x)
	
	def __iter__(self):
		for key, value in self.get_way_point().items():
			yield key, value
	
	def __str__(self):
		return 'x:{0}, y:{1}, z:{2}, yaw:{3}'.format(self.x, self.y, self.z, self.yaw)

	def get_way_point(self):
		return dict( x = self.x[0], y = self.y[0], z = self.z[0], yaw = self.yaw[0] )

	def update_way_points( self, pose ):
		error = 0
		for key, value in self:
			error += ( value - getattr(pose, key) )**2
		error = math.sqrt(error)

		if error < ERROR_WAY_POINT: 
			self.x.pop(0)
			self.y.pop(0)
			self.z.pop(0)
			self.yaw.pop(0)
	
	@property
	def x(self):
		return self.properties['x']

	@x.setter
	def x(self, value):
		self.properties['x'] = value

	@x.deleter
	def x(self):
		del self.properties['x']

	@property
	def y(self):
		return self.properties['y']

	@y.setter
	def y(self, value):
		self.properties['y'] = value

	@y.deleter
	def y(self):
		del self.properties['y']

	@property
	def z(self):
		return self.properties['z']

	@z.setter
	def z(self, value):
		self.properties['z'] = value

	@z.deleter
	def z(self):
		del self.properties['z']

	@property
	def yaw(self):
		return self.properties['yaw']

	@yaw.setter
	def yaw(self, value):
		self.properties['yaw'] = value

	@yaw.deleter
	def yaw(self):
		del self.properties['yaw']

class RosLauncher(object):
	"""docstring for RosLauncher"""
	def __init__(self):
		super(RosLauncher, self).__init__()
		self.launchers = dict()
		self.processes = dict()
	
	def init_node(self, package, node_type, name=None, namespace='/', 
		machine_name=None, args='', respawn=False, remap_args=None,
		env_args=None, output=None, cwd=None, launch_prefix=None, 
		required=False, filename='<unknown>'):
		return roslaunch.core.Node(
			package, node_type, name, namespace, 
			machine_name, args, respawn, remap_args,
			env_args, output, cwd, launch_prefix, 
			required, filename)

	def launch_node(self, node, name):
		if name in self.launchers.keys():
			self.kill_node(name)
		self.launchers[name] = roslaunch.scriptapi.ROSLaunch()
		self.launchers[name].start()
		self.processes[name] = self.launchers[name].launch(node)
	
	def launch_nodes(self, node_list, group_name):
		if group_name in self.launchers.keys():
			self.kill_nodes(name)

		self.launchers[group_name] = roslaunch.scriptapi.ROSLaunch()
		self.launchers[group_name].start()
		self.processes[group_name] = [self.launchers[group_name].launch(node) for node in node_list]

	def kill_node(self, name):
		try:
			self.launchers[name].stop()
			del self.launchers[name]
			del self.processes[name]
		except KeyError:
			pass

	def kill_nodes(self, group_name):
		try:
			self.launchers[group_name].stop()
			del self.launchers[group_name]
			del self.processes[group_name]
		except KeyError:
			pass

	def set_param(self, node_name, param_name, param):
		rospy.set_param( '/{0}/{1}'.format(node_name, param_name), param )
		
class MainWindow(QtGui.QMainWindow, object):
	"""docstring for MainWindow"""
	def __init__(self, **kwargs):
		QtGui.QMainWindow.__init__( self )
		self.timeZero = rospy.get_time()
		self.commander = JoystickController()

		self.timer = dict()

		self.client = dict(
			controller = dynamic_reconfigure.client.Client('controller'), 
			state_estimation = dynamic_reconfigure.client.Client('state_estimation'), 
			)

		self.update_dict = dict(
			recieveEstimatedPose = False,
			recieveEstimatedVel = False,
			recieveDesiredPose = False,
			recieveDesiredVel = False,
			recieveCmdVel = False,
			)
	
		self.pushButtons = dict()
		self.radioButtons = dict()
		self.radioButtons['groups'] = OrderedDict()
		self.comboBox = dict()

		self.infoLabel = OrderedDict()

		self.actions = dict()
		self.menus = dict()

		self.groupBox = dict( )

		self.layouts = dict()

		
		

		self.rosLauncher = RosLauncher()

		self.variableList = ['x', 'y','z','yaw']

		self.initMainLayout()
		self.plot = ArDronePlotCanvas(self.layouts['plotDisplay'], width=5, height=4, dpi=100)
		self.cameraImage = CameraCanvas()

		self.initDirectCommandsLayout()
		self.initCameraDisplayLayout()
		self.initPlotDisplayLayout()
		self.initStateDisplayLayout()


		self.initMenuBar()
		self.initStatusBar()


		timer = QtCore.QTimer(self)
		QtCore.QObject.connect(timer, QtCore.SIGNAL("timeout()"), self.update)
		timer.start(REFRESH_GUI_TIME)

		self.subscriber = dict( 
			cmd_vel = rospy.Subscriber('cmd_vel', Twist, callback = self.recieveCmdVel),
			#state = rospy.Subscriber('ardrone/estimated_pose', Odometry, callback = self.recieveStateEstimation), 
			estimated_pose = rospy.Subscriber('ardrone/sensor_fusion/pose', QuadrotorPose, callback = self.recieveEstimatedPose),
			estimated_velocity = rospy.Subscriber('ardrone/sensor_fusion/velocity', QuadrotorPose, callback = self.recieveEstimatedVel),
			estimated_quaternion = rospy.Subscriber('ardrone/sensor_fusion/quaternion', QuaternionMsg, callback = self.recieveEstimatedQuaternion),
			desired_pose = rospy.Subscriber('ardrone/trajectory/pose', QuadrotorPose, callback = self.recieveDesiredPose),
			desired_velocity = rospy.Subscriber('ardrone/trajectory/velocity', QuadrotorPose, callback = self.recieveDesiredVel),
			commanded_velocity = rospy.Subscriber('cmd_vel', Twist, callback=self.recieveCmdVel)
			)

		#self.setWindowState(QtCore.Qt.WindowMaximized)
	
	def initMainLayout( self ):
		mainWidget = QtGui.QWidget( self )
		mainLayout = QtGui.QGridLayout()

		
		self.layouts['main'] = mainLayout
		self.layouts['cameraDisplay'] = QtGui.QVBoxLayout()
		self.layouts['stateDisplay'] = QtGui.QVBoxLayout()
		self.layouts['plotDisplay'] = QtGui.QVBoxLayout()
		self.layouts['directCommands'] = QtGui.QVBoxLayout()

		mainLayout.addLayout( self.layouts['cameraDisplay'] , 0, 0 )
		mainLayout.addLayout( self.layouts['plotDisplay'] , 0, 1 )
		mainLayout.addLayout( self.layouts['stateDisplay'] , 1, 0 )
		mainLayout.addLayout( self.layouts['directCommands'] , 1, 1 )
		"""
		mainLayout.addLayout( self.layouts['plotDisplay'] , 0, 0, 3, 2 )
		mainLayout.addLayout( self.layouts['stateDisplay'] , 3, 0 )
		
		mainLayout.addLayout( self.layouts['directCommands'], 3,1 )
		"""
		mainWidget.setLayout(mainLayout)

		self.setCentralWidget( mainWidget )

		self.setWindowTitle('Ar.Drone Controller GUI')

		x, y, w, h = 400, 200, 800, 700
		self.setGeometry(x, y, w, h)

		self.setWindowIcon(QtGui.QIcon(QtGui.QPixmap( 
			os.path.dirname(os.path.abspath(__file__))+'/figs/freeflight.png')) 
		)
		self.centerOnScreen()

	def centerOnScreen (self):
		'''centerOnScreen() Centers the window on the screen.'''
		resolution = QtGui.QDesktopWidget().screenGeometry()
		self.move((resolution.width() / 2) - (self.frameSize().width() / 2),
			(resolution.height() / 2) - (self.frameSize().height() / 2))

		#self.setLayout(main_layout)

	def initStateDisplayLayout( self ):
		#timer = QtCore.QTimer(self)
		#QtCore.QObject.connect(timer, QtCore.SIGNAL("timeout()"), self.update_state)
		#timer.start(1000)

		layout = OrderedDict( )

		#self.groupBoxList = ['generalInfo', 'position', 'velocity', 'setPoint', 'cmdVel' ]
		
		self.groupBox['generalInfo'] = QtGui.QGroupBox("General Info")
		layout['generalInfo'] = QtGui.QHBoxLayout()
		self.infoLabel['generalInfo'] = OrderedDict()
		self.infoLabel['generalInfo']['batteryPercent'] = QtGui.QLabel( "Battery Percent: ")
		#self.infoLabel['generalInfo']['controller'] = QtGui.QLabel( "Controller Type: ")

		self.radioButtons['groups']['controllerType'] = QtGui.QButtonGroup()
		self.radioButtons['controllerType'] = OrderedDict()
		self.radioButtons['controllerType']['velocityLocal'] = QtGui.QRadioButton("Local Velocity", self.groupBox['generalInfo'], clicked=self.onControllerSelect)
		self.radioButtons['controllerType']['velocityLocal'].setChecked(False)
		self.radioButtons['controllerType']['velocityGlobal'] = QtGui.QRadioButton("Global Velocity", self.groupBox['generalInfo'], clicked=self.onControllerSelect)
		self.radioButtons['controllerType']['velocityGlobal'].setChecked(False)
		self.radioButtons['controllerType']['positionGlobal'] = QtGui.QRadioButton("Position Control", self.groupBox['generalInfo'], clicked=self.onControllerSelect)
		self.radioButtons['controllerType']['positionGlobal'].setChecked(False)

		layout['generalInfo'].addWidget( self.infoLabel['generalInfo']['batteryPercent'] )

		for button in self.radioButtons['controllerType'].values():
			#QtCore.QObject.connect(button, QtCore.SIGNAL("clicked()"), self.onControlSource)
			layout['generalInfo'].addWidget( button )
			self.radioButtons['groups']['controllerType'].addButton( button )


		self.groupBox['velocity'] = QtGui.QGroupBox("Estimated Speed")
		layout['velocity'] = QtGui.QHBoxLayout()

		self.infoLabel['velocity'] = OrderedDict()
		self.infoLabel['velocity']['x'] = QtGui.QLabel( "x: ")
		self.infoLabel['velocity']['y'] = QtGui.QLabel( "y: ")
		self.infoLabel['velocity']['z'] = QtGui.QLabel( "z: ")
		self.infoLabel['velocity']['yaw'] = QtGui.QLabel( "vyaw: ")


		self.groupBox['position'] = QtGui.QGroupBox("Estimated Position")
		layout['position'] = QtGui.QHBoxLayout()
		self.infoLabel['position'] = OrderedDict()
		self.infoLabel['position']['x'] = QtGui.QLabel( "x: ")
		self.infoLabel['position']['y'] = QtGui.QLabel( "y: ")
		self.infoLabel['position']['z'] = QtGui.QLabel( "z: ")
		self.infoLabel['position']['yaw'] = QtGui.QLabel( "yaw: ")


		self.groupBox['setPoint'] = QtGui.QGroupBox("Position Set Point")
		layout['setPoint'] = QtGui.QHBoxLayout()
		self.infoLabel['setPoint'] = OrderedDict()
		self.infoLabel['setPoint']['x'] = QtGui.QLabel( "x: ")
		self.infoLabel['setPoint']['y'] = QtGui.QLabel( "y: ")
		self.infoLabel['setPoint']['z'] = QtGui.QLabel( "z: ")
		self.infoLabel['setPoint']['yaw'] = QtGui.QLabel( "yaw: ")

		self.groupBox['cmdVel'] = QtGui.QGroupBox("Ar.Drone Command [roll, pitch, vz, vyaw]")
		layout['cmdVel'] = QtGui.QHBoxLayout()

		self.infoLabel['cmdVel'] = OrderedDict()
		self.infoLabel['cmdVel']['x'] = QtGui.QLabel("x: {0:.2f}".format( self.commander.velocity['x'] ) )
		self.infoLabel['cmdVel']['y'] = QtGui.QLabel("y: {0:.2f}".format( self.commander.velocity['y'] ) )
		self.infoLabel['cmdVel']['z'] = QtGui.QLabel("z: {0:.2f}".format( self.commander.velocity['z'] ) )
		self.infoLabel['cmdVel']['yaw'] = QtGui.QLabel("yaw: {0:.2f}".format( self.commander.velocity['yaw'] ) )


		for group, dictionaries in self.infoLabel.items():
			for label in dictionaries.values():
				layout[group].addWidget( label )

			self.groupBox[group].setLayout( layout[group] )

			self.layouts['stateDisplay'].addWidget( self.groupBox[group] )

		self.unSelectRadioButtons( 'controllerType' )
		#self.updateSetPoint()

	def initDirectCommandsLayout( self ):
		self.groupBox['directCommand']  = QtGui.QGroupBox("Direct Commands")
		layout = QtGui.QVBoxLayout()

		
		self.pushButtons['directCommand'] = OrderedDict()
		self.pushButtons['directCommand']['land'] = QtGui.QPushButton("Land", self, clicked=self.commander.land)
		self.pushButtons['directCommand']['takeOff']  = QtGui.QPushButton("Take Off", self, clicked=self.commander.take_off)	
		self.pushButtons['directCommand']['reset']  = QtGui.QPushButton("Reset", self, clicked=self.commander.reset) 	
		self.pushButtons['directCommand']['stop']  = QtGui.QPushButton("Stop", self, clicked=self.commander.stop) 

		self.pushButtons['directCommand']['land'].setMinimumHeight(60);

		for button in self.pushButtons['directCommand'].values():
			layout.addWidget( button )

		self.groupBox['directCommand'].setLayout(layout)

		self.groupBox['controlSource'] = QtGui.QGroupBox("Control Source")
		layout = QtGui.QGridLayout()


		
		self.radioButtons['groups']['controlSource'] = QtGui.QButtonGroup()
		
		self.radioButtons['controlSource'] = dict()
		self.radioButtons['controlSource']['PS3'] = QtGui.QRadioButton("PS3 Joystick", self.groupBox['controlSource'], clicked=self.onControlSource)
		self.radioButtons['controlSource']['PS2']  = QtGui.QRadioButton("Regular Jostick", self.groupBox['controlSource'], clicked=self.onControlSource)
		#self.radioButtons['Wii']  = QtGui.QRadioButton("Wii Joystick", self.controlSourceGroup)
		self.radioButtons['controlSource']['keyboard']  = QtGui.QRadioButton("Keyboard", self.groupBox['controlSource'], clicked=self.onControlSource)
		self.radioButtons['controlSource']['textFile']  = QtGui.QRadioButton("Text File Way Points", self.groupBox['controlSource'], clicked=self.onControlSource)
		self.radioButtons['controlSource']['function']  = QtGui.QRadioButton("Python Function", self.groupBox['controlSource'], clicked=self.onControlSource)
		self.radioButtons['controlSource']['dialog']  = QtGui.QRadioButton("Dialog Window", self.groupBox['controlSource'], clicked=self.onControlSource)



		i = 0
		for button in self.radioButtons['controlSource'].values():
			#QtCore.QObject.connect(button, QtCore.SIGNAL("clicked()"), self.onControlSource)
			layout.addWidget( button, i/3, i % 3 )
			i += 1

			self.radioButtons['groups']['controlSource'].addButton( button )

		self.groupBox['controlSource'].setLayout(layout)

		self.layouts['directCommands'].addWidget( self.groupBox['directCommand'] )
		self.layouts['directCommands'].addWidget( self.groupBox['controlSource'] )
	
	def initCameraDisplayLayout( self ):
		#pass 
		
		self.groupBox['camera']  = QtGui.QGroupBox("Ar.Drone Camera Display")
		layout = QtGui.QVBoxLayout()
		cameraSelectLayout = QtGui.QHBoxLayout()


		
		self.radioButtons['cameraSelect'] = dict()
		self.radioButtons['cameraSelect']['frontCamera'] = QtGui.QRadioButton("Front Camera", clicked=self.onCameraSelect)
		self.radioButtons['cameraSelect']['frontCamera'].setChecked(True)
		self.radioButtons['cameraSelect']['bottomCamera'] = QtGui.QRadioButton("Bottom Camera", clicked=self.onCameraSelect)
		self.radioButtons['cameraSelect']['bottomCamera'].setChecked(False)

		cameraSelectLayout.addWidget(self.radioButtons['cameraSelect']['frontCamera'])
		cameraSelectLayout.addWidget(self.radioButtons['cameraSelect']['bottomCamera'])

		layout.addLayout(cameraSelectLayout)
		layout.addWidget(self.cameraImage)

		self.groupBox['camera'].setLayout(layout)
		self.layouts['cameraDisplay'].addWidget( self.groupBox['camera']  )
		

	def initPlotDisplayLayout( self ):
		self.groupBox['plot'] = QtGui.QGroupBox("Ar.Drone Plotter")
		layout = QtGui.QVBoxLayout()

		self.pushButtons['plot'] = dict()
		self.pushButtons['plot']['selectVars'] = QtGui.QPushButton("Select Plot Variables", self)
		self.pushButtons['plot']['selectVars'] .clicked.connect( self.selectPlotVariables )
		
		layout.addWidget( self.pushButtons['plot']['selectVars']  )
		layout.addWidget( self.plot )
		self.groupBox['plot'] .setLayout(layout)
		self.layouts['plotDisplay'].addWidget( self.groupBox['plot']  )

	def initMenuBar( self ):
		
		menubar = QtGui.QMenuBar()

		self.menus['file'] = menubar.addMenu('&File')
		self.actions['file'] = dict()
		self.actions['file']['exit'] = QtGui.QAction(QtGui.QIcon('exit.png'), '&Exit', self)        
		self.actions['file']['exit'].setShortcut('Ctrl+Q')
		self.actions['file']['exit'].setStatusTip('Exit application')
		self.actions['file']['exit'].triggered.connect(self._closeEvent)
		#self.actions['file']['exit'].triggered.connect(QtGui.qApp.quit)

		recordMenu = self.menus['file'].addMenu( "&Record" )

		self.actions['file']['record'] = OrderedDict()
		self.actions['file']['record']['navdata'] = QtGui.QAction( "Record &Navdata", self,checkable=True, triggered=partial(self.recordVar, "record_navdata", 'navdata' ))
		self.actions['file']['record']['cmd_vel'] = QtGui.QAction( "Record Command &Velocity", self,checkable=True, triggered=partial(self.recordVar, "record_cmd_vel", 'cmd_vel' ))
		self.actions['file']['record']['imu'] = QtGui.QAction( "Record &Imu", self,checkable=True, triggered=partial(self.recordVar, "record_imu", 'imu' )) 
		#self.actions['file']['record']['image'] = QtGui.QAction( "Record &Image", self,checkable=True, triggered=partial(self.recordVar, "record_image", 'image'))
		#self.actions['file']['record']['sonar'] = QtGui.QAction( "Record &Sonar", self,checkable=True, triggered=partial(self.recordVar, "record_sonar", 'sonar'))
		self.actions['file']['record']['mag'] = QtGui.QAction( "Record &Magnetometer", self,checkable=True, triggered=partial(self.recordVar, "record_mag", 'mag'))
		self.actions['file']['record']['gps'] = QtGui.QAction( "Record &GPS", self,checkable=True, triggered=partial(self.recordVar, "record_gps", 'gps'))
		
		self.actions['file']['record']['sensor_fusion_pose'] = QtGui.QAction( "Record Sensor Fusion &Pose", self,checkable=True, triggered=partial(self.recordVar, "record_sensor_fusion_pose", 'sensor_fusion_pose' ))
		self.actions['file']['record']['sensor_fusion_velocity'] = QtGui.QAction( "Record Sensor Fusion &Velocity", self,checkable=True, triggered=partial(self.recordVar, "record_sensor_fusion_velocity", 'sensor_fusion_velocity' ))
		self.actions['file']['record']['sensor_fusion_quaternion'] = QtGui.QAction( "Record Sensor Fusion &Quaternion", self,checkable=True, triggered=partial(self.recordVar, "record_sensor_fusion_quaternion", 'sensor_fusion_quaternion' ))

		self.actions['file']['record']['trajectory_pose'] = QtGui.QAction( "Record Desired &Pose", self,checkable=True, triggered=partial(self.recordVar, "record_trajectory_pose", 'trajectory_pose' ))
		self.actions['file']['record']['trajectory_velocity'] = QtGui.QAction( "Record Desired &Velocity", self,checkable=True, triggered=partial(self.recordVar, "record_trajectory_velocity", 'trajectory_velocity' ))
		
		self.actions['file']['record2'] = OrderedDict()
		self.actions['file']['record2']['chirp'] = QtGui.QAction( "Record &Chirp", self,checkable=True, triggered=self.recordChirp )
		self.actions['file']['record2']['position_control'] = QtGui.QAction( "Record &Position Control", self,checkable=True, triggered=self.recordPositionControl )

		#self.actions['file']['record']['sensorfusion'] = QtGui.QAction( "Record &Estimation", self,checkable=True, triggered=partial(self.recordVar, "record_sensorfusion", 'sensorfusion'))
		#self.actions['file']['record']['trajectory'] = QtGui.QAction( "Record Commanded &Trajectory", self,checkable=True, triggered=partial(self.recordVar, "record_trajectory", 'trajectory'))
		
		self.actions['file']['record']['recordAll'] = QtGui.QAction( "Record &All", self,checkable=True, triggered=self.recordAll)

		for action in self.actions['file']['record'].values():
			recordMenu.addAction( action )

		for action in self.actions['file']['record2'].values():
			recordMenu.addAction( action )


		self.menus['setUp'] = menubar.addMenu('&Set Up')
		self.actions['setUp'] = OrderedDict()

		self.actions['setUp']['initialCondition'] = QtGui.QAction( "Setup &Inditial Conditions", self, triggered = self.selectInitialConditions)
		self.actions['setUp']['initialCondition'].setShortcut('Ctrl+I')
		self.actions['setUp']['outdoor'] = QtGui.QAction( "&Outdoor or Indoor Environment", self, triggered = self.selectEnvironment)
		self.actions['setUp']['shell']  = QtGui.QAction( "&Select Shell", self, triggered=self.selectShell)
		self.actions['setUp']['recordUSB']  = QtGui.QAction( "&Record USB Enable", self,checkable=True)
		self.actions['setUp']['recordUSB'].triggered.connect(lambda: self.commander.record( self.actions['recordUSB'].isChecked() ) )
		self.actions['setUp']['controllerWizard'] = QtGui.QAction( "Controller Wizard", self, triggered=self.selectControllerWizard )		
		self.actions['setUp']['controllerWizard'].setShortcut('Ctrl+C+W')
		#self.actions['setUp']['toggleCam']  = QtGui.QAction( "&Toggle Camera", self,checkable=False, triggered=self.onToggleCam )

		self.menus['calibration'] = menubar.addMenu('&Calibration')
		self.actions['calibration'] = OrderedDict()

		self.actions['calibration']['flatTrim'] = QtGui.QAction( "&Flat Trim", self, triggered=self.commander.flat_trim )
		self.actions['calibration']['imuRecal'] = QtGui.QAction( "&Imu Re-Calibration", self, triggered=self.onImuCalibration )
		self.actions['calibration']['gps'] = QtGui.QAction( "&GPS Calibrate", self, triggered=self.commander.calibrate_gps )
		self.actions['calibration']['imu'] = QtGui.QAction( "&Imu Calibrate", self, triggered=self.commander.calibrate_imu )

		self.menus['animation'] = menubar.addMenu('&Animations')
		self.actions['animation'] = dict()

		self.actions['animation']['ledAnimation'] = QtGui.QAction( "&Led Animation", self, triggered=self.selectLedAnimation )
		self.actions['animation']['flightAnimation']  = QtGui.QAction( "&Flight Animation", self, triggered=self.selectFlightAnimation )

		"""
		self.menus['controller'] = menubar.addMenu('Controller')
		self.actions['controller'] = dict()
		#self.actions['controller']['controlType'] = QtGui.QAction( "&Control Type", self, triggered=self.selectControlType )
		"""
		self.menus['command'] = menubar.addMenu('Command')
		self.actions['command'] = OrderedDict()
		
		self.actions['command']['chirp'] = QtGui.QAction( "&Default Chirp", self, triggered=self.selectChirp )
		self.actions['command']['openLoopSignals'] = QtGui.QAction( "&Open Loop Command", self, triggered=self.selectSignal )

		self.menus['serial'] = menubar.addMenu('Serial')
		self.actions['serial']= OrderedDict()
		
		arduinoMenu = self.menus['serial'].addMenu( "&Arduino" )
		self.actions['serial']['arduino'] = OrderedDict()
		self.actions['serial']['arduino']['start'] = QtGui.QAction( "&Start Arduino Connection", self, triggered=partial(self.onArduino,'start') )
		self.actions['serial']['arduino']['reset'] = QtGui.QAction( "&Reset Arduino Connection", self, triggered=partial(self.onArduino,'reset') )
		self.actions['serial']['arduino']['close'] = QtGui.QAction( "&Stop Arduino Connection", self, triggered=partial(self.onArduino,'close') )

		for action in self.actions['serial']['arduino'].values():
			arduinoMenu.addAction( action )

		xbeeMenu = self.menus['serial'].addMenu( "&X-Bee" )
		self.actions['serial']['xbee'] = OrderedDict()
		self.actions['serial']['xbee']['start'] = QtGui.QAction( "&Start X-Bee Connection", self, triggered=partial(self.onXbee,'start') )
		self.actions['serial']['xbee']['reset'] = QtGui.QAction( "&Reset X-Bee Connection", self, triggered=partial(self.onXbee,'reset') )
		self.actions['serial']['xbee']['close'] = QtGui.QAction( "&Stop X-Bee Connection", self, triggered=partial(self.onXbee,'close') )

		for action in self.actions['serial']['xbee'].values():
			xbeeMenu.addAction( action )


		for menu in self.actions.keys():
			for action in self.actions[menu].values():
				try:
					self.menus[menu].addAction(action)
				except TypeError:
					pass

		self.setMenuBar(menubar)

	def initStatusBar( self ):
		self.statusLabel = QtGui.QLabel("Ar.Drone State: <b> {0} </b> ".format( self.commander.get_state() ))	
		self.statusBar().addWidget( self.statusLabel ) 

	def onArduino( self, action, clicked ):
		if action == 'start':
			baud, port, ok  = ArduinoSetupDialog().getData( )
			if ok:
				node = self.rosLauncher.init_node( "rosserial_python", "serial_node.py",'arduino' )
				self.rosLauncher.launch_node( node, 'rosserial' )

				self.rosLauncher.set_param( 'arduino','boaud', baud)
				self.rosLauncher.set_param( 'arduino','port', baud)

				node = self.rosLauncher.init_node( "ardrone_control", "arduino_translator.py",'arduino_translator' )
				self.rosLauncher.launch_node( node, 'arduino_translator' )

			else:
				return 

		elif action == 'reset':
			self.onArduino( self, 'close' )
			self.onArduino( self, 'start' )

		elif action == 'stop':
			self.rosLauncher.kill_node('rosserial')
			self.rosLauncher.kill_node('arduino_translator')
	
	def onXbee( self, action, clicked):
		if action == 'start':
			baud, port, ok  = XbeeSetupDialog().getData( )
			if ok:
				self.rosLauncher.set_param( 'xbee','baud', baud)
				self.rosLauncher.set_param( 'xbee','port', port)

				node = self.rosLauncher.init_node( "ardrone_control", "gps_reciever.py",'gps_reciever' )
				self.rosLauncher.launch_node( node, 'gps_reciever' )

			else:
				return 

		elif action == 'reset':
			self.onXbee( self, 'close' )
			self.onXbee( self, 'start' )

		elif action == 'stop':
			self.rosLauncher.kill_node('gps_reciever')

	def onCameraSelect( self ):
		if self.radioButtons['cameraSelect']['frontCamera'].isChecked():
			self.commander.cam_select(0)
		elif self.radioButtons['cameraSelect']['bottomCamera'].isChecked():
			self.commander.cam_select(1)

	def onControlSource( self ):
		if self.radioButtons['controlSource']['PS3'].isChecked():
			#self.commander.joy = PS3(); 
			self.launchJoy( PS3() )
		elif self.radioButtons['controlSource']['PS2'].isChecked():
			#self.commander.joy = PS2(); 
			self.launchJoy( PS2() )			
		elif self.radioButtons['controlSource']['function'].isChecked():
			self.killJoy( )
			fileName = QtGui.QFileDialog.getOpenFileName(self,
				"Select Trajectory Function", 
				"All Files (*);;Python Files (*.py)")
			if fileName != "":
				foo = imp.load_source('trajectory', str(fileName) )
				self.trajectoryStart( foo.Trajectory( rospy.get_time() ) )
			else:
				self.unSelectRadioButtons('controlSource')
		elif self.radioButtons['controlSource']['textFile'].isChecked():
			self.killJoy( )
			fileName= QtGui.QFileDialog.getOpenFileName(self,
				"Select Way Point Text File",
				"Text Files (*.txt)")
			if fileName != "":
				fopen = open(fileName, 'r')
				x = []
				y = []
				z = []
				yaw = []
				for line in fopen:
					sp = [float(a) for a in line.rstrip().split(',')]
					x.append( sp[0] )
					y.append( sp[1] )
					z.append( sp[2] )
					yaw.append( sp[3] * math.pi / 180.0 )
				
				self.wayPointStart( WayPoints(x,y,z,yaw) )
			else:
				self.unSelectRadioButtons('controlSource')	
		elif self.radioButtons['controlSource']['dialog'].isChecked():
			self.killJoy( )
			set_point_type, sp,ok = InputSetPointDialog().getValues()
			if ok:
				if set_point_type == 'Position':
					for key, value in sp.items():
						if key == 'yaw':
							self.commander.position[key] = value * math.pi / 180.0 
						else:
							self.commander.position[key] = value 
				else:
					for key, value in sp.items():
						if key == 'yaw':
							self.commander.position[key] = value * math.pi / 180.0 
						else:
							self.commander.position[key] = value  

			self.unSelectRadioButtons('controlSource')
		elif self.radioButtons['controlSource']['keyboard'].isChecked():
			self.killJoy( )

	def onControllerSelect( self ):
		if self.radioButtons['controllerType']['positionGlobal'].isChecked():
			self.commander.control_on()
		else:
			self.commander.control_off()

			if self.radioButtons['controllerType']['velocityGlobal'].isChecked():
				self.commander.glob( )
			else:
				self.commander.local( )

	def onImuCalibration( self ):
		msgBox = QtGui.QMessageBox(QtGui.QMessageBox.Warning, "Warning", "Caution, experimental!", QtGui.QMessageBox.NoButton, self)
		msgBox.addButton("Continue Anyways", QtGui.QMessageBox.AcceptRole)
		msgBox.addButton("Don't Calibration", QtGui.QMessageBox.RejectRole)
		if msgBox.exec_() == QtGui.QMessageBox.AcceptRole:
			self.commander.imu_calibration()

	def selectInitialConditions( self ):
		name, position, sensors, ok = InitialConditionDialog().getData()
				
		if ok:
			params = dict( name = name )
			params.update( position )
			params.update( sensors )
			try:
				config = self.client['state_estimation'].update_configuration(params)
			except rospy.ServiceException as exc:
				rospy.logwarn("Check Syntax \nService did not process request: " + str(exc))
			#set position, velocity and sensors in state_estimation node

	def selectEnvironment( self ):
		reply = QtGui.QMessageBox.question(self, "Environment Selection",
					"Is it flying Outdoor?",
					QtGui.QMessageBox.Yes | QtGui.QMessageBox.No )
		if reply == QtGui.QMessageBox.Yes:
			self.commander.outdoor()
		elif reply == QtGui.QMessageBox.No:
			self.commander.indoor()

	def selectShell( self ):
		reply = QtGui.QMessageBox.question(self, "Shell Parameter Selection",
					"Is it wearing a shell?",
					QtGui.QMessageBox.Yes | QtGui.QMessageBox.No | QtGui.QMessageBox.Cancel)
		if reply == QtGui.QMessageBox.Yes:
			self.commander.fly_with_shell()
		elif reply == QtGui.QMessageBox.No:
			self.commander.fly_without_shell()

	def selectLedAnimation( self ):
		led_animation_list = [ "blink green red", "blink green", "blink red", "blink orange", 
		"snake green red", "fire", "standard", "red", "green", "red snake", "blank", 
		"left green right red", "left red right green", "blink standard"
		]

		led_animation, ok = QtGui.QInputDialog.getItem(self, "Select Led Animation", "Led Animation", led_animation_list, 0, False)
		animation_type = led_animation_list.index(led_animation)

		freq, ok2 = QtGui.QInputDialog.getInteger(self, 
					"Animation Frequency:", "Frequency [Hz]:", 5, 0, 60, 1)

		duration, ok3 = QtGui.QInputDialog.getInteger(self, 
					"Duration:", "time [sec]:", 5, 0, 60, 1)

		if ok & ok2 & ok3:
			self.commander.led_animation( animation_type, freq, duration )

	def selectFlightAnimation( self ):
		flight_animation_list = ["phi -30 deg", "phi +30 deg", "theta -30 deg", "theta +30 deg",
		"theta +20 deg yaw +200 deg", "theta +20 deg yaw -200 deg", 
		"turnaround", "turnaround go down",
		"yaw shake", "yaw dance", "phi dance", "theta dance", "vz dance" "wave", 
		"phi theta mixed", "double phi theta mixed" ,
		"flip ahead", "flip behind", "flip left", "flip right", ]


		flight_animation, ok = QtGui.QInputDialog.getItem(self, "Select Flight Animation", "Flight Animation", flight_animation_list, 0, False)
		animation_type = flight_animation_list.index( flight_animation)
		if animation_type >= 15:
			msgBox = QtGui.QMessageBox(QtGui.QMessageBox.Warning, "Warning", "Caution, this is a dangerous animation!", QtGui.QMessageBox.NoButton, self)
			msgBox.addButton("Continue Anyways", QtGui.QMessageBox.AcceptRole)
			msgBox.addButton("Stop Animation", QtGui.QMessageBox.RejectRole)
			if msgBox.exec_() == QtGui.QMessageBox.AcceptRole:
				ok = ok & True
			else:
				ok = False
		if ok:
			 self.commander.flight_animation( animation_type, 0)

	def selectControllerWizard( self ):
		controller_type, direction, digital, Ts, parameters, ok = ControllerSetupWizard().getValues()
		
		if ok:
			params = {'type': controller_type, 'direction': direction, 'digital': digital, 'Ts': Ts}
			params.update(parameters)
			try:
				config = self.client['controller'].update_configuration(params)
			except rospy.ServiceException as exc:
				rospy.logwarn("Check Syntax \nService did not process request: " + str(exc))
		
	def selectChirp( self ):
		coordinate_list = ["x", "y", "z", "yaw"]
		direction, ok = QtGui.QInputDialog.getItem(self, "Select Chirp Direction", "Coordinate", coordinate_list, 0, False)
		direction = str(direction)
		if ok:
			self.commander.signal_init( self.commander.default_chirp_data(direction) )
			self.plot.reset()
			if direction == 'z':
				self.plot.activePlots = [ ( 'velocity' ,direction ) , ('position', direction)]	
			else:
				self.plot.activePlots = [ ( 'velocity' ,direction ) , ('cmdVel', direction)]		
			self.plot.createPlotHandles()
			self.unSelectRadioButtons('controlSource')

	def selectSignal( self ):
		data, ok  = SignalSetupDialog().getData( )
		if ok:
			self.commander.signal_init( data )
			self.plot.activePlots = [ 'Velocity {0}'.format(data.direction), 'Command Velocity {0}'.format(data.direction)]
			self.plot.createPlotHandles()
			self.unSelectRadioButtons('controlSource')

	def selectPlotVariables(self ): 

		self.plot.selectVariablesDialog()
		"""
		variables, ok = QtGui.QInputDialog.getInteger(self, 
			"Number of Variables", "Number of Variables to Plot:", 0, 0, 8, 1)
		if not ok:
			return 

		self.plot.reset( )
		plotVariableList = []
		for variable in range(variables):
			item, ok = QtGui.QInputDialog.getItem(self, 
					"Enter the Variable you want to Plot ", 
					"Variable {0}:".format(variable + 1), self.plot.inactivePlots, 
					0, False)
			if ok:
				self.plot.activatePlot( str(item) )

		self.plot.createPlotHandles()
		"""
				
			#set variables in 
	
	def killJoy(self):
		self.commander.joy = [ ];
		self.rosLauncher.kill_node("joy")

	def launchJoy(self, joystick ):
		port, ok = QtGui.QInputDialog.getText(self, "Joystick Setup Dialog", " Select Port:", QtGui.QLineEdit.Normal,"/dev/input/js0")

		if ok:
			#"/dev/input/js0" 
			self.commander.joy = joystick
			node = self.rosLauncher.init_node( "joy", "joy_node",'joystick' )
			self.rosLauncher.launch_node( node, 'joystick' )
			self.rosLauncher.set_param( "joy", "dev", port)
		#else:
			#self.unSelectRadioButtons('controlSource')
	
	def recordVar( self, scriptName , key ):
		if self.actions['file']['record'][key].isChecked():
			node = self.rosLauncher.init_node( "ardrone_control", scriptName, scriptName )
			print node.name
			self.rosLauncher.launch_node( node, scriptName )
		else:
			self.rosLauncher.kill_node( scriptName )

	def recordAll( self ):
		if self.actions['file']['record']['recordAll'].isChecked():
			for key, action in self.actions['file']['record'].items():
				if key is not 'recordAll':
					action.setChecked(True)
					self.recordVar( 'record_{0}'.format(key), key )

		else:
			for key, action in self.actions['file']['record'].items():
				if key is not 'recordAll':
					action.setChecked(False)
					self.recordVar( 'record_{0}'.format(key), key )
	
	def recordChirp(self):
		keys = ['cmd_vel', 'navdata', 'imu']
		if self.actions['file']['record2']['chirp'].isChecked():
			for key in keys:
				action = self.actions['file']['record'][key]
				action.setChecked(True)
				self.recordVar( 'record_{0}'.format(key), key )
		else:
			for key in keys:
				action = self.actions['file']['record'][key]
				action.setChecked(False)
				self.recordVar( 'record_{0}'.format(key), key )

	def recordPositionControl(self):
		keys = ['sensor_fusion_pose', 'sensor_fusion_velocity', 'trajectory_pose', 'trajectory_velocity']

		if self.actions['file']['record2']['position_control'].isChecked():
			for key in keys:
				action = self.actions['file']['record'][key]
				action.setChecked(True)
				self.recordVar( 'record_{0}'.format(key), key )
		else:
			for key in keys:
				action = self.actions['file']['record'][key]
				action.setChecked(False)
				self.recordVar( 'record_{0}'.format(key), key )

	def unSelectRadioButtons( self, group_name ):
		self.radioButtons['groups'][group_name].setExclusive(False)
		for button in self.radioButtons[group_name].values():
			button.setChecked(False)		  
		self.radioButtons['groups'][group_name].setExclusive(True)  
		#self.onControlSource()

	def _closeEvent(self, clicked):
		reply = QtGui.QMessageBox.question(self, 'Message',
			"Are you sure to quit?", QtGui.QMessageBox.Yes | 
			QtGui.QMessageBox.No, QtGui.QMessageBox.No)

		if reply == QtGui.QMessageBox.Yes:
			self.commander.land()
			QtGui.qApp.quit()

	def closeEvent(self, event):
		reply = QtGui.QMessageBox.question(self, 'Message',
			"Are you sure to quit?", QtGui.QMessageBox.Yes | 
			QtGui.QMessageBox.No, QtGui.QMessageBox.No)

		if reply == QtGui.QMessageBox.Yes:
			self.commander.land()
			event.accept()
		else:
			event.ignore() 

	def keyPressEvent(self, event):
		key = event.key()
		if event.isAutoRepeat():
			return

		if key == QtCore.Qt.Key_Backspace:
			self.commander.stop()
		elif key == QtCore.Qt.Key_Escape:
			self.commander.land()

		if self.radioButtons['controlSource']['keyboard'].isChecked():
			if key == QtCore.Qt.Key_Return:
				self.commander.take_off()
			elif key == QtCore.Qt.Key_Space:
				self.commander.reset()
			elif key == QtCore.Qt.Key_C:
				self.commander.control_on()
			elif key == QtCore.Qt.Key_F:
				self.commander.control_off()


			## Velocity Commands
			if key == QtCore.Qt.Key_A:
				self.commander.vyaw( -1 )
			elif key == QtCore.Qt.Key_D:
				self.commander.vyaw( +1 )
			elif key == QtCore.Qt.Key_S:
				self.commander.vz( -1 )
			elif key == QtCore.Qt.Key_W:
				self.commander.vz( +1)
			elif key == QtCore.Qt.Key_I:
				self.commander.vx( +1 )
			elif key == QtCore.Qt.Key_K:
				self.commander.vx( -1 )
			elif key == QtCore.Qt.Key_L:
				self.commander.vy( +1 )
			elif key == QtCore.Qt.Key_J:
				self.commander.vy( -1 )

			## Position Commands
			modifiers = QtGui.QApplication.keyboardModifiers()
			if modifiers == QtCore.Qt.ShiftModifier:
				if key == QtCore.Qt.Key_Left:
					self.commander.yaw( -1 )
				elif key == QtCore.Qt.Key_Right:
					self.commander.yaw( +1 )
				elif key == QtCore.Qt.Key_Down:
					self.commander.z( -1 )
				elif key == QtCore.Qt.Key_Up:
					self.commander.z( +1 )
			else:
				if key == QtCore.Qt.Key_Left:
					self.commander.y( -1 )
				elif key == QtCore.Qt.Key_Right:
					self.commander.y( +1 )
				elif key == QtCore.Qt.Key_Down:
					self.commander.x( -1 )
				elif key == QtCore.Qt.Key_Up:
					self.commander.x( +1 )

	def keyReleaseEvent(self, event):
		key = event.key()
		if event.isAutoRepeat():
			return
		if self.radioButtons['controlSource']['keyboard'].isChecked():
			if key == QtCore.Qt.Key_A or key == QtCore.Qt.Key_D:
				self.commander.vyaw( 0 )
			elif key == QtCore.Qt.Key_S or key == QtCore.Qt.Key_W:
				self.commander.vz( 0 )
			elif key == QtCore.Qt.Key_I or key == QtCore.Qt.Key_K:
				self.commander.vx( 0 )
			elif key == QtCore.Qt.Key_L or key == QtCore.Qt.Key_J:
				self.commander.vy( 0 )

	def wayPointStart(self, wayPointObject):
		self.wayPoint = wayPointObject
		self.commander.control_on()
		self.timer.update( way_point_update = rospy.Timer(rospy.Duration( COMMAND_TIME ), self.wayPointUpdate, oneshot=False) ) 
		
		for button in self.radioButtons['controlSource'].values():
			button.setEnabled(False)

	def wayPointUpdate(self, time_data):
		try:
			if len(self.wayPoint)>0:
				for key, value in self.wayPoint:
					self.commander.position[key] = value
			else:
				self.wayPointStop()
		except AttributeError:
			self.timer['way_point_update'].shutdown()
			
	def wayPointStop(self):
		self.timer['way_point_update'].shutdown()
		del self.wayPoint
		self.commander.stop()
		rospy.loginfo("Way Point Trajectory Ended")

		for button in self.pushButtons['directCommand'].values():
			button.setEnabled(True)
		for button in self.radioButtons['controlSource'].values():
			button.setEnabled(True)
		self.unSelectRadioButtons('controlSource')

	def trajectoryStart(self, trajectory_object):
		self.trajectory = trajectory_object
		
		duration, ok = QtGui.QInputDialog.getInteger(self, 
					"Duration:", "time [sec]:", 5, 0, 120, 1)

		self.trajectory.t0 = rospy.get_time()
		self.commander.control_on()
		self.timer.update( trajectory_update = rospy.Timer(rospy.Duration( COMMAND_TIME ), self.trajectoryUpdate, oneshot=False) )
		rospy.Timer(rospy.Duration( duration ), self.trajectoryStop, oneshot=True) 

		for button in self.radioButtons['controlSource'].values():
			button.setEnabled(False)

	def trajectoryUpdate(self, time_data ):
		t = rospy.get_time()

		self.commander.position['x'] = self.trajectory.x( t )
		self.commander.position['y'] = self.trajectory.y( t )
		self.commander.position['z'] = self.trajectory.z( t )
		self.commander.position['yaw'] = self.trajectory.yaw( t )

		self.commander.velocity['x'] = self.trajectory.vx( t )
		self.commander.velocity['y'] = self.trajectory.vy( t )
		self.commander.velocity['z'] = self.trajectory.vz( t )
		self.commander.velocity['yaw'] = self.trajectory.vyaw( t )

	def trajectoryStop(self, *args):
		self.timer['trajectory_update'].shutdown()
		self.commander.stop()
		rospy.loginfo("Trajectory Ended")

		for button in self.pushButtons['directCommand'].values():
			button.setEnabled(True)
		for button in self.radioButtons['controlSource'].values():
			button.setEnabled(True)
		self.unSelectRadioButtons('controlSource')
	
	def recieveCmdVel(self, twist):
		if not self.update_dict[ inspect.stack()[0][3] ] :
			return
		self.update_dict[ inspect.stack()[0][3] ] = False
		pose_msg = QuadrotorPose()
		pose_msg.x = twist.linear.x 
		pose_msg.y = twist.linear.y 
		pose_msg.z = twist.linear.z
		pose_msg.yaw = twist.angular.z

		self.parsePoseMsg( 'cmdVel', pose_msg )
	
	def recieveEstimatedPose( self, pose ):
		if not self.update_dict[ inspect.stack()[0][3] ] :
			return

		self.update_dict[ inspect.stack()[0][3] ] = False

		self.parsePoseMsg( 'position', pose )

		if hasattr(self, 'wayPoint'):
			self.wayPoint.update_way_points( pose )

	def recieveEstimatedVel( self, vel ):
		if not self.update_dict[ inspect.stack()[0][3] ] :
			return

		self.update_dict[ inspect.stack()[0][3] ] = False

		self.parsePoseMsg( 'velocity', vel )

	def recieveDesiredPose( self, pose ):
		if not self.update_dict[ inspect.stack()[0][3] ] :
			return

		self.update_dict[ inspect.stack()[0][3] ] = False

		self.parsePoseMsg( 'setPoint', pose )

	def recieveDesiredVel( self, vel ):
		pass

	def recieveEstimatedQuaternion(self, quaternion_msg):
		t = rospy.get_time() - self.timeZero
		for key, plot in self.plot.plots['quaternion'].items():
			plot.append(t, getattr(quaternion_msg.quaternion, key))

	def parsePoseMsg(self, attribute, pose_msg):
		t = rospy.get_time() - self.timeZero
		for key in self.infoLabel[attribute].keys():
			if key == 'yaw':
				self.infoLabel[attribute][key].setText("{0}: {1}".format( key, getattr(pose_msg, key) * 180.0 / math.pi ) )
			else:
				self.infoLabel[attribute][key].setText("{0}: {1:.2f}".format( key, getattr(pose_msg, key) ) )
			self.plot.plots[attribute][key].append( t, getattr(pose_msg, key) )

	def updateControllerState( self ):
		if self.commander.state == ArDroneStates.Landed:
			self.unSelectRadioButtons('controllerType')
		else:
			self.radioButtons['controllerType']['positionGlobal'].setChecked(self.commander.controller_state )
			self.radioButtons['controllerType']['velocityLocal'].setChecked(not self.commander.controller_state and self.commander.local_controller)
			self.radioButtons['controllerType']['velocityGlobal'].setChecked(not self.commander.controller_state and not self.commander.local_controller)

	def updateGeneralInfo( self ):
		self.infoLabel['generalInfo']['batteryPercent'].setText("Battery Percent: {0:.2f}".format(self.commander.battery))
		self.statusLabel.setText( "Ar.Drone State: <b> {0} </b> ".format( self.commander.get_state() )  )

	def updateCamera( self ):
		self.cameraImage.updateCamera()

	def updatePlot( self ):
		self.plot.updateFigure()

	def update( self ):
		#self.updateSetPoint()
		
		self.updateGeneralInfo()
		self.updatePlot()
		self.updateCamera()
		self.updateControllerState()

		if self.commander.state == ArDroneStates.Landed :
			for button in self.pushButtons['directCommand'].values():
				button.setEnabled(True)
			for button in self.radioButtons['controlSource'].values():
				button.setEnabled(True)
			#self.unSelectRadioButtons('controlSource')

			if hasattr(self, 'wayPoint'):
				del self.wayPoint

		for key in self.update_dict.keys():
			self.update_dict[key] = True

def main():

	rospy.init_node('ardrone_commander', anonymous = True)

	app = QtGui.QApplication(sys.argv)

	

	main_window = MainWindow( )
	main_window.show()

	
	sys.exit(app.exec_())

	rospy.spin()

if __name__ == "__main__": main()