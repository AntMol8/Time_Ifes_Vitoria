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
	#Receives coordinates x and y from the gps
	global pos_x
	global pos_y
	global pos_z
	pos_x = GPS_data.latitude
	pos_y = GPS_data.longitude
	pos_z = GPS_data.altitude


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
	

	i = 0
	#Avoids repeated data
	if(memory != Hokuyo_data.reading and pub!= 1):

		#Empties arrays with previous data
		data_x[:] = []
		data_y[:] = []
		data_z[:] = []
		
		#The radius of the data interpreted is reduced to 1 m of distance from the reference. Signal informs if none of the data is in range and shuts down the function
		signal = 0
		print(Hokuyo_data)
		while i< len(Hokuyo_data.reading):
			print(Hokuyo_data.reading[i]**2 + (Hokuyo_data.reading[i+1]**2))
			if(((Hokuyo_data.reading[i]**2) + (Hokuyo_data.reading[i+1]**2) <= 1) and (round(Hokuyo_data.reading[i+1]) == 0)):
				data_x.append(-Hokuyo_data.reading[i])
				data_y.append(-Hokuyo_data.reading[i+1])
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

		#Sees if any of the data received is immediately to the side of the Hokuyo
		for i in range(len(data_y)):
			if((data_y[i] > -0.1) and (data_y[i] < 0.1)):
				#In case there is, checks if the structure has the shape of the top of the roll
				for j in range(len(data_x)):
					if((((data_x[i] - data_x[j]) <= 0.06) and ((data_x[i] - data_x[j]) >= 0.04)) and (abs(data_y[i] - data_y[j]) > 0.03)):
						#flag that tells UR5 that a roll was found

						pub.data = 1
						flag.publish(pub)
						return
	memory = Hokuyo_data.reading

		


if __name__ == "__main__":
	try:	
		pathfinder()
	except rospy.ROSInterruptException:
		print "Exception occured. Shutting down."
