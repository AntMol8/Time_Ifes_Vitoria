#!/usr/bin/env python
import numpy as np
import rospy
import math
from sensor_msgs.msg import NavSatFix
from rosi_defy.msg import HokuyoReading
from std_msgs.msg import Float32MultiArray
from PIL import Image as IMG

#Variables that control position
data_x = []
data_y = []
data_z = []
check_status = 0
current_position = [0] * 3
k = 0.4
pos_x, pos_y, pos_z, x, y = 0.0, 0.0, 0.0, 0.0, 0.0
msg = rospy.Publisher('/canaldecomunicao', Float32MultiArray, queue_size=10)
#Map creation
mapa = np.zeros([1000, 1000], dtype = np.uint8)
mapa[..., 500] = 255
map_x=[]
map_y=[]
map_z=[]
memory=()

def createMap():
	rospy.sleep(2)
	global data_x
	global data_y
	global data_z
	global pos_x
	global pos_y
	global pos_z
	global map_x
	global map_y
	global map_z
	
	map_x[:] = []
	map_y[:] = []
	map_z[:] = []

	resolution = 0.01
	for i in range(len(data_x)):
		map_x.append(int((data_x[i]+pos_x)/resolution) + 500)
		map_y.append(int((data_y[i]+pos_y)/resolution) + 500)
		map_z.append(int((data_z[i]+pos_z)/2))
		
	for i in range(len(map_x)):
		if (map_x[i] < 1000 and map_x[i] >= 0) and (map_y[i] < 1000 and map_y[i] >= 0):
			mapa[map_y[i],map_x[i]] = 255

	imagem = IMG.fromarray(mapa, 'L')
	imagem.save('caminho1.png', "PNG")

	

def pathfinder():

	#Inits node and declares subscribers
	rospy.init_node('HOKUYO_MAPPING', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, position)
	rospy.Subscriber('/sensor/hokuyo', HokuyoReading, callback)
	
	rospy.spin()

def position(GPS_data):
	#Receives coordinates x and y from the gps
	global pos_x
	global pos_y
	global pos_z
	pos_x = GPS_data.latitude
	pos_y = GPS_data.longitude
	pos_z = GPS_data.altitude
	print("AAAA")
	print(pos_y)

def callback(Hokuyo_data):
	#Based on the Hokuyo_data and preset distance k, it compares de average distance to the value preset, defining the coordinates it will send to the script movement
	global data_x
	global data_y
	global data_z
	global pos_x
	global pos_y
	global k
	global check_status
	global msg
	global memory
	
	msg = rospy.Publisher('/canaldecomunicao', Float32MultiArray, queue_size=10)
	pub = Float32MultiArray()

	i = 0
	j = 0
	distance = 0
	check_status = 0
	
	if(memory != Hokuyo_data.reading):
		#Empties arrays with previous data
		data_x[:] = []
		data_y[:] = []
		data_z[:] = []

		while i< len(Hokuyo_data.reading):
			data_x.append(Hokuyo_data.reading[i])
			data_y.append(Hokuyo_data.reading[i+1])
			data_z.append(Hokuyo_data.reading[i+2])
			i = i + 3

		for i in range(len(data_x)):
			distance = math.sqrt(data_x[i]**2 + data_y[i]**2) + distance

		distance = distance/(len(data_x))
		
		#First scenario - robot is beside the object
		for i in range(len(data_x)):
			print(data_x[i])
			if((data_x[i]<=0.2) and (data_x[i]>=-0.2)):
				check_status = 1

		#Second scenario, object is in front of robot
		for i in range(len(data_y)):
			if((data_y[i]<=0.5) and (data_y[i]>=-0.5)):
				check_status = 2

		if(check_status == 1):
			result_distance = k - distance #Might not be distance
			pos_y = pos_y + result_distance

		if(check_status == 2):
			pos_y = pos_y + 0.5
		
		#Simply to illustrate that the robot always moves forward
		#if(check_status == 0):
			#pos_y = pos_y		
		
		a=(pos_x,pos_y)
		print(a)
		print(check_status)
		print(result_distance)
		print(pos_y)
		if(check_status != 0):
			pub.data = a
			msg.publish(pub)
		createMap()
	memory = Hokuyo_data.reading

		


if __name__ == "__main__":
	try:	
		pass
		#pathfinder()
	except rospy.ROSInterruptException:
		print "Exception occured. Shutting down."
