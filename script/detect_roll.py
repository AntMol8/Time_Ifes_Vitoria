#!/usr/bin/env python
import numpy as np
import rospy
import math
from sensor_msgs.msg import NavSatFix
from rosi_defy.msg import HokuyoReading
from std_msgs.msg import Float32MultiArray
from PIL import Image as IMG

#Variable declaration
# x, y and z data from the Hokuyo
data_x = []
data_y = []
data_z = []

# variables that hold the GPS coordinates
pos_x, pos_y = 0.0, 0.0

# holds the last output from the Hokuyo
memory = ()

# indicates that the beggining or the end of the roll has been identified
roll = False

# holds the coordinates of all the previous rolls
previous_roll_x =  []
previous_roll_y = []

# rolls are published in groups of 4 coordinates: (x coordinate - first edge of the roll, y coordinate - first edge of the roll, x coordinate - second edge of the roll, y coordinate - second edge of the roll)
# n makes sure that there are 4 coordinates before they are sent
n = 0

# temporarily holds the values of the beggining and end of a roll before they are published
coordinates = []

# checks if program should look for beggining or end of the roll
condition = 0

#marks the gps position at the moment the first edge of a roll is detected
mark_spot = 100

def pathfinder(): # iits node; inits Hokuyo and GPS subscriber; inits publisher that sends roll coordinates
	global pub
	global flag

	# iits node and declares subscribers
	rospy.init_node('DETECT_ROLL', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, position)
	rospy.Subscriber('/sensor/hokuyo', HokuyoReading, callback)
	
	# flag publisher sends rolls coordinates to be printed on a map
	flag = rospy.Publisher('/detect_roll', Float32MultiArray, queue_size=1)
	pub = Float32MultiArray()
	pub.data = 0
	rospy.spin()

def position(GPS_data): #receives data from GPS. Controls the publishing of roll coordinates
	
	global n
	global previous_roll_x
	global previous_roll_y
	global roll
	#Receives coordinates x and y from the gps
	global pos_x
	global pos_y
	global pub
	global flag
	global coordinates
	
	pos_x = GPS_data.latitude
	pos_y = GPS_data.longitude
	
	# checks if roll has been detected
	if (roll):
		
		# rolls always have coordinates that correspond to the edges of the conveyor belt.
		# checks if robot is either to the right or the left of the robot, and assigns y pos of the roll detected accordingly
	        if(pos_y > 1):
	                position_y = 1.245
	                position_x = pos_x + 0.136
	        elif (pos_y < -1):
	        	position_y = -1.245
	        	position_x = pos_x - 0.136
	       	else:
	       		print("error")
	       		return

		print(position_x, position_y)

		# checks if a beginning edge has been detected
		#if it has, assures that the edge detected is a closing edge. In case no, ignores data
		if condition != n:
			n += 1
			
			# adds coordinates of the edge detected to coordinates, where they are held temporarily
			coordinates.append(position_x)
			coordinates.append(position_y)
			print coordinates
			
			# if there are 4 coordinates, moves to publishing
			if (n == 2):
				pub.data = coordinates
				# checks if roll has been previoulsy detected
				for i in range(0, len(previous_roll_x), 2):
					if (coordinates[0] + 0.2 > previous_roll_x[i] and coordinates[0] - 0.2 < previous_roll_x[i]) and (coordinates[1] + 0.2 > previous_roll_y[i] and coordinates[1] - 0.2 < previous_roll_y[i]):
						print("AA")
						# in case roll has been detected, ends function and erases coordinates
						roll = False
						del coordinates[:]
						n = 0
						return
				#publishes roll's coordinates and adds data to previously detected rolls
				print(pub)
				flag.publish(pub)
				previous_roll_x.append(coordinates[0])
				previous_roll_y.append(coordinates[1])
				previous_roll_x.append(coordinates[2])
				previous_roll_y.append(coordinates[3])
				del coordinates[:]
				n = 0
				
		roll = False


def callback(Hokuyo_data):
	#Based on the Hokuyo_data the script identifies the presence of a roll
	global data_x
	global data_y
	global data_z
	global pos_x
	global pos_y
	global flag
	global memory
	global pub
	global roll
	global condition
	global mark_spot
	
	i = 0
	#Avoids repeated data
	if(memory != Hokuyo_data.reading and pub != 1):
		memory = Hokuyo_data.reading

		#Empties arrays with previous data
		data_x[:] = []
		data_y[:] = []
		data_z[:] = []
		
		#The radius of the data interpreted is reduced to 1 m of distance from the reference. Signal informs if none of the data is in range and shuts down the function
		signal = 0
		while i < len(Hokuyo_data.reading):
			if (((Hokuyo_data.reading[i]**2) + (Hokuyo_data.reading[i + 1]**2) <= 1) and (round(Hokuyo_data.reading[i +1 ]) == 0)):
				data_x.append(Hokuyo_data.reading[i])
				data_y.append(Hokuyo_data.reading[i + 1])
				data_z.append(Hokuyo_data.reading[i + 2])
				signal = 1
			i = i + 3

		if (signal == 0):
			return
	
		#Rounds up data to simplify calculations
		for i in range(len(data_x)):
			data_x[i] = round(data_x[i], 2)
			data_y[i] = round(data_y[i], 2)
			data_z[i] = round(data_z[i], 2)
			
		print("---------------------")

		#Sees if any of the data received is immediately to the side of the Hokuyo
		for i in range(len(data_y)):
			if ((data_y[i] > -0.01) and (data_y[i] < 0.01)):
				#In case there is, checks if the structure has the shape of the top of the roll
				j = i
				while j < (len(data_y) - 1):
					# checks for the form of the edge of the roll (circular)
					if (abs(data_x[i] - data_x[j]) <= 0.1 and abs(data_x[i] - data_x[j]) >= 0.04 and abs(data_y[i] - data_y[j]) > 0.01 and abs(data_y[i] - data_y[j]) < 0.8):
						if (data_x[i] - data_x[j]) < 0:
							# checks if edge is an opening edge (crescent curvature) or end edge (decrescent curvature) 
							if condition == 0 and abs(pos_x - mark_spot) > 0.18 :
								print("ABRE")
								print(i, j)
								
								# gets coordinates from gps at the moment of detection
								# makes program look for end edge
								mark_spot = pos_x
								condition = 1
								roll = True
							break
						elif (data_x[i] - data_x[j]) > 0 and condition == 1:
							# only enters this condition after opening edge has been detected
							if condition == 1:
								# checks if distance from opening to end edges is greater than 5 cm
								# if not, just ignores end coordinate
								if abs(pos_x - mark_spot) > 0.05:
									print("FECHA")
									print(i, j)
									#sets program to look for opening edges
									condition = 0
									roll = True
							break
					j += 1
				else:
					continue
				break
						#flag that tells UR5 that a roll was found

		if (condition == 1):
			print mark_spot
			print pos_x
			
			# if the robot has moved for more than 15 cm after detecting opening edge, determines end edge is its current positioning
			if (abs(pos_x - mark_spot) > 0.15):
				print("FECHA")
				condition = 0
				roll = True
			return
			
if __name__ == "__main__":
	try:	
		pathfinder()
	except rospy.ROSInterruptException:
		print "Exception occured. Shutting down."
