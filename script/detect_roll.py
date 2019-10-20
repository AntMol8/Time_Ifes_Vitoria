#!/usr/bin/env python
import numpy as np
import rospy
import math
from sensor_msgs.msg import NavSatFix
from rosi_defy.msg import HokuyoReading
from std_msgs.msg import Float32MultiArray
from PIL import Image as IMG

#Variable declaration
data_x = []
data_y = []
data_z = []
pos_x, pos_y, pos_z, x, y = 0.0, 0.0, 0.0, 0.0, 0.0
memory=()
roll = False
previous_roll_x =  []
previous_roll_y = []
n = 0
coordinates = []
condition = 0
mark_spot = 100

	

def pathfinder():
	global pub
	global flag
	#Inits node and declares subscribers
	rospy.init_node('DETECT_ROLL', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, position)
	rospy.Subscriber('/sensor/hokuyo', HokuyoReading, callback)
	flag = rospy.Publisher('/detect_roll', Float32MultiArray, queue_size=1)
	pub = Float32MultiArray()
	pub.data=0
	rospy.spin()

def position(GPS_data):
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
	check = 0
	if (roll):
	        if(pos_y > 1):
	                position_y = 1.245
	                position_x = pos_x + 0.136
	        elif (pos_y < -1):
	        	position_y = -1.245
	        	position_x = pos_x - 0.136
	       	else:
	       		print("error")
	       		return
	       	
		for i in range(0, len(previous_roll_x),2):
			try:
				if (coordinates[0] + 0.2 > previous_roll_x[i] and coordinates[0] - 0.2 < previous_roll_x[i]) and (coordinates[1] + 0.2 > previous_roll_y[i] and coordinates[1] - 0.2 < previous_roll_y[i]):
					print("AA")
					
			except:
				break
		print(position_x, position_y)
		if condition != n:
			n += 1
			coordinates.append(position_x)
			coordinates.append(position_y)
			print coordinates
			if(n ==2):
				pub.data = coordinates
				for i in range(0, len(previous_roll_x),2):
					if (coordinates[0] + 0.2 > previous_roll_x[i] and coordinates[0] - 0.2 < previous_roll_x[i]) and (coordinates[1] + 0.2 > previous_roll_y[i] and coordinates[1] - 0.2 < previous_roll_y[i]):
						print("AA")
						roll = False
						coordinates = []
						return

				print(pub)
				flag.publish(pub)
				previous_roll_x.append(coordinates[0])
				previous_roll_y.append(coordinates[1])
				previous_roll_x.append(coordinates[2])
				previous_roll_y.append(coordinates[3])
				coordinates = []
				n=0
				
		roll = False


def callback(Hokuyo_data):
	#Based on the Hokuyo_data the script identifies the presence of a roll
	global data_x
	global data_y
	global data_z
	global pos_x
	global pos_y
	global k
	global check_status
	global flag
	global memory
	global pub
	global roll
	global condition
	global mark_spot
	

	i = 0
	#Avoids repeated data
	if(memory != Hokuyo_data.reading and pub!= 1):
		memory = Hokuyo_data.reading

		#Empties arrays with previous data
		data_x[:] = []
		data_y[:] = []
		data_z[:] = []
		
		#The radius of the data interpreted is reduced to 1 m of distance from the reference. Signal informs if none of the data is in range and shuts down the function
		signal = 0
		while i< len(Hokuyo_data.reading):
			if(((Hokuyo_data.reading[i]**2) + (Hokuyo_data.reading[i+1]**2) <= 1) and (round(Hokuyo_data.reading[i+1]) == 0)):
				data_x.append(Hokuyo_data.reading[i])
				data_y.append(Hokuyo_data.reading[i+1])
				data_z.append(Hokuyo_data.reading[i+2])
				signal = 1
			i = i + 3
		if(signal == 0):
			return
	
		#Rounds up data to simplify calculations
		for i in range(len(data_x)):
			data_x[i] = round(data_x[i], 2)
			data_y[i] = round(data_y[i], 2)
			data_z[i] = round(data_z[i], 2)
			
		print("---------------------")

		#Sees if any of the data received is immediately to the side of the Hokuyo
		for i in range(len(data_y)):
			if((data_y[i] > -0.01) and (data_y[i] < 0.01)):
				#In case there is, checks if the structure has the shape of the top of the roll
				j = i
				while j < (len(data_y) -1):
					if(abs(data_x[i] - data_x[j]) <= 0.1 and abs(data_x[i] - data_x[j]) >= 0.04 and abs(data_y[i] - data_y[j]) > 0.01 and abs(data_y[i] - data_y[j]) < 0.8):
						if (data_x[i] - data_x[j]) < 0:
							if condition == 0 and abs(pos_x - mark_spot) > 0.18 :
								print("ABRE")
								print(i, j)
								mark_spot = pos_x
								condition = 1
								roll = True
							break
						elif (data_x[i] - data_x[j]) > 0 and condition == 1:
							if condition == 1:
								if abs(pos_x - mark_spot) > 0.05:
									print("FECHA")
									print(i, j)
									condition = 0
									roll = True
							break
					j += 1
				else:
					continue
				break
						#flag that tells UR5 that a roll was found
		if(condition == 1):
			print mark_spot
			print pos_x
			if(abs(pos_x - mark_spot) > 0.15):
				print("FECHA")
				condition = 0
				roll = True
			return

if __name__ == "__main__":
	try:	
		pathfinder()
	except rospy.ROSInterruptException:
		print "Exception occured. Shutting down."
