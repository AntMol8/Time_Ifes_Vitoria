# ROSI CHALLENGE - TIME_IFES_VITORIA
This repository contains the package to perform the ROSI CHALLENGE using ROS operational system and V-REP as a simulator. It was developed on Python 2.7 and may not work in other versions.

## How to install and running
	The installation and running can be done by the following steps:		
	1) Clone and download the repository package in the <catkin_ws>/src/
	2) Compile your catkin_ws using catkin build
	3) Open terminal and run roscore and vrep in order
	4) Open challenge_scenario.ttt on vrep
	5) Go to the scripts folder and run the chmod +x command on all script files (Example: $ chmod +x *.py)
	6) Insert on the terminal $ roslaunch Time_Ifes_Vitoria challenge.launch

## What it does
The launch package is composed of 11 scripts, that are:

- ### create_trajectory.py:

Receives coordinates from 'main_control' and publishes the parameters of a hiberbolic tangent to 'trajectory_parameters' so it can control the robot's movement.

- ### detect_roll.py:

Receives information from GPS and Hokuyo. Identifies rolls based on the Hokuyo's output and publishes their estimated coordinates to be used as reference on the map generation and on the touch routine.

- ### kinect_compensation.py:

Receives information about the robot's orientation from IMU and controls the kinect joint in order to keep it parallel to the ground.

- ### main_control.py:

Receives information from 'object_control' and 'ur5_control' and commands other function of the robot, enabling and disabling parts of other scripts, such as 'ur5_control' and 'create_trajectory', in order to perform the tasks as intended, for instance when the robot should execute the touching routine and when it should alter its trajectory.

- ### movement.py: 

Gets coordinates of where the robot should go in a straight line by linking the robot's position and its objective, the angle of the joints of the wheels and the robot's speed and transforms it into movement by a proportional closed control system. Generates map, marking the fires' positions and displaying it in red color. Generates map, marking the fires', the rolls' and the conveyor belt's positions, the robot's trajectory, and displaying it according to the legend.

- ### object_control.py:

Receives coordinates from 'object_detection' and 'object_velodyne' and publishes the coordinates of the object so the robot can bypass it or the restricted area, so the robot can enter it, and go back to its normal trajectory to 'main_control'.

- ### object_detection.py:

Receives information from the kinect depth camera, calculates the distance to any object in front of it and publishes coordinates to 'object_control' so the robot can bypass it.

- ### object_velodyne.py:

Receives information from velodyne, calculates the distance to objects in front of it and by its side and publishes coordinates to 'object_control' so the robot can either bypass it, when the object is in front of it, or go back to its normal trajectory, when the object is by its side.

- ### trajectory_parameters.py:

Receives the parameters of a function from 'create_trajectory' and publishes coordinates, speed and arms angle to 'movement' with the objective of making it perform a hyperbolical tangent trajectory when bypassing an object.

- ### ur5_control.py:

Is controlled by 'main_control' and determines the angular position of the UR5 joints, also executing routines of touching parts of the conveyor belt.

- ### ur5Camera.py:

Receives information from the RGB channel of UR5's tool camera and estimates the coordinates of a fire and publishes it to 'movement', that generates the map.
			
## Map
The map is a 6000x1000 image named 'mapa.png'. After the execution of the program, the image can be found in either the Home folder or in the .ros folder:
		
- ### Map Legend:

	- Black - Ground / Out of bounds
	
	- Blue Line - ROSI trajectory
	
	- Green - Rolls' estimated position
	
	- Red - Fire 
	
	- White - Conveyor Belt

		
