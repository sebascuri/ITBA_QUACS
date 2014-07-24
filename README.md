# QUACS: A ROS controller for AR.Drone
==========

## Table of Contents

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Modules](#odules)
	- [State Estimation](#state-estimation)
	- [Controller](#controller)
	- [State Handling](#state-handling)
- [Usage](#usage)
	- [Keyboard Command](#keyboard-command)
	- [Joystick Command](#joystick-command)
	- [Text File Command](#text-file-command)
- [Modifications](#modifications)
- [To do](#to-do)

## Introduction
This is a repository of a ROS package that controls the parrot AR.DRONE.

## Dependencies
This package depends on:

* ROS Hydro Distribution ( http://wiki.ros.org/hydro/Installation/Ubuntu )
* Ardrone Autonomy Package ( https://github.com/AutonomyLab/ardrone_autonomy )
* Dynamic Reconfigure ( http://wiki.ros.org/dynamic_reconfigure )
* Joy ( http://wiki.ros.org/joy )
* Numpy ( http://www.numpy.org/ )
* Scipy ( http://scipy.org/ ) 
* PyQt4 ( http://pyqt.sourceforge.net/Docs/PyQt4/installation.html )
* UTM Converter ( https://pypi.python.org/pypi/utm )

## Modules
### State Estimation
This module estimates quadrotor state using: 
* Digital Low-Pass (Butterworth) Filtering of Sensors, 
* Kalman Filtering to fusion sensors and Odometry to predict positions. 
* Quaternion Non-Linear filters to fusion Gyroscopes, Accelerometers and Magnetometers

Publishes results in message topic 'ardrone/sensorfusion/navdata' with type Odometry

### Controller
This module controlls the position of the quadrotor. A PID or Digital Transfer Function controller can be applied

Commands Drone using message topic 'cmd_vel' with type Twist

### GUI
There is a User Interface to Control the Drone. 

### Keyboard Command
When executed this module A GUI window will open, while active:


* Press J (L) key to increase (decrease) Y-coordinate goal or velocity according to the controller state
* Press I (K) key to increase (decrease) X-coordinate goal or velocity according to the controller state
* Press W (S) key to increase (decrease) Z-coordinate goal or velocity according to the controller state
* Press A (D) key to increase (decrease) Yaw-coordinate goal or velocity according to the controller state
* Press C (F) arrow to turn controller on (off)

### Joystick Command
When executed this module the joy_node will be running:

* Press 'X' for Drone to take-off
* Press 'Circle' for Drone to land
* Press 'Triangle' for Drone to Reset
* Move Right Analog towards Right (Left) to increase (decrease) Y-coordinate goal or velocity according to the controller state
* Move Right Analog towards Up (Down) to increase (decrease) X-coordinate goal or velocity according to the controller state
* Move Left Analog towards Right (Left) to increase (decrease) Yaw-coordinate goal or velocity according to the controller state
* Move Left Analog towards Up (Down) to increase (decrease) Z-coordinate goal or velocity according to the controller state
* Press 'Square' ('Select') arrow to turn controller on (off)


## Usage
### GUI
To run the GUI with the complete nodal structure run:
```bash
$ roslaunch ardrone_control ardrone_gui.launch
```

## To do

- Add Camera Processing Capabilities
- Add a ZPK controller user interface
- Add a SOS controller subdivision
