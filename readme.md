# ROSI CHALLENGE - TIME_IFES_VITORIA
This repository contains the package to perform the ROSI CHALLENGE using ROS operational system and V-REP as a simulator. It was developed on Python 2.7 and may not work in other versions.

## How to install and running
	The installation and running can be done by the following steps:		
	1) Clone and download the repository package in the <catkin_ws>/src/ folder
	2) Compile your catkin_ws using catkin build
	3) Run the file challenge.launch

## What it does
The launch package is composed of 5 scripts, that are:

- ### movement: 

Gets coordinates of where the robot should go in a straight line by linking the robot's position and its objective, the angle of the joints of the wheels and the robot's speed and transforms it into movement by a proportional closed control system. Generates map, marking the fires' positions and displaying it in red color.
		
- ### cpmo: 

Commands other functions of the robot, enabling and disabling parts of other scripts in order to perform the tasks intended.

- ### rosi_joy: 

Receives the parameters of a function from 'rosim' and publishes coordinates, speed and wheel angle to 'movement' with the objective of making it perform a hyperbolical tangent trajectory when making bypassing and object.

- ### rosim: 

Receives coordinates from 'cpmo' and publishes the parameters of a hiberbolic function to 'rosi_joy' so it can control the robot's movement.

- ### proof_of_concept_control: 

Is controlled by cpmo and determines the angular position of the UR5, also regulating the action of touching parts of the conveyor belt.
			
## Map
The map is a 6000x1000 image named 'mapa.png'. After the execution of the program, the image can be found in either the Home folder or in the .ros folder:
		
- Map Legend:
	
	- Red - Fire 
	
	- Blue - Robot's trajectory
	
	- White - Conveyor Belt (esteira)
	
	- Black - Ground / Out of bounds

		
