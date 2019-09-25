#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import cv2
import numpy as np
import math
from struct import unpack
orientation = -1 #baseado na orientacao
inn=46
gps_x_global = 0; gps_y_global = 0; gps_z_global = 0
imu_x_global=0; imu_y_global=0; imu_z_global=0
mapa_global = np.zeros((7000, 3000), np.uint8)
mapa_global[...,1500]=127; mapa_global[6000,...]=127; mapa_global[4670,...]=127
x1=int(-51.825*100+6000); x2=int(-2.15*100+6000)
y1=int(1.245*100+1500);y2=int(-1.425*100+1500)
cv2.rectangle(mapa_global, (y2, x2), (y1, x1), (250), -1)
def GPS(GPS_data):
	global gps_x_global
	global gps_y_global
	global gps_z_global
	global inn
	gps_x_global = GPS_data.latitude
	gps_y_global = GPS_data.longitude
	gps_z_global = GPS_data.altitude
	inn=inn+1
	#print 'GPS'
def IMU(Imu_data): #velodyne tem frequencia de 0.7-1 hz, testar com o movimento da rosi a 2 r/s de velocidade da roda
	global imu_x_global
	global imu_y_global
	global imu_z_global
	global orientation
	global inn
	inn=inn+1
	a = Imu_data.orientation.w
	b = Imu_data.orientation.x
	c = Imu_data.orientation.y
	d = Imu_data.orientation.z
	ref = math.sqrt((1-b**2-c**2)/2)
	if (np.absolute(d) > ref):
		orientation = -1 		# eh false
	else:
		orientation = 1
	imu_x_global = np.arctan((2*(a*b + c*d)) / (a*a - b*b - c*c + d*d))
	imu_y_global = np.arcsin(2 * (b*d - a*c))
	imu_z_global = np.arctan((2*(a*d + c*b)) / (a*a + b*b - c*c - d*d))

def VELODYNE(velodyne_data):
	global mapa_global
	global gps_x_global
	global gps_y_global
	global gps_z_global
	global imu_x_global
	global imu_y_global
	global imu_z_global
	global orientation
	global inn
	print inn
	if(inn>=50):
		inn=0
		gps_x = gps_x_global
		gps_y = gps_y_global
		gps_z = gps_z_global
		imu_x = imu_x_global
		imu_y = imu_y_global
		imu_z = imu_z_global
		angulo_y = 15*np.pi/180-imu_y
		mapa = np.zeros((10000, 3000), np.uint8)
		mapa[...,1500]=127; mapa[5000,...]=127; mapa[5100,...]=127; 		mapa[5200,...]=127; mapa[5300,...]=127; mapa[5400,...]=127;
		for i in range(0, len(velodyne_data.data), 12):
			value_x = unpack('f', velodyne_data.data[i:i+4])[0]
			value_z = unpack('f', velodyne_data.data[i+8:i+12])[0]
			value_y = unpack('f', velodyne_data.data[i+4:i+8])[0]
			velodyne_coordinates = np.array([[value_x], [value_y], [value_z]])
			rotation_matrix_z = np.array([ [np.cos(imu_z), -np.sin(imu_z), 0], [np.sin(imu_z), np.cos(imu_z), 0], [0, 0, 1] ]) #esqueci que imu_z eh um angulo de euler e tem retricoes ----------------------------------
			rotation_matrix_x = np.array([ [1, 0, 0], [0, np.cos(imu_x), -np.sin(imu_x) ], [0, np.sin(imu_x), np.cos(imu_x)] ])
			rotation_matrix_y = np.array([ [np.cos(angulo_y), 0, np.sin(angulo_y)], [0, 1, 0], [-np.sin(angulo_y), 0, np.cos(angulo_y)] ])
			temp_coordinates = np.dot(rotation_matrix_y,velodyne_coordinates)
			coordinates = np.dot(rotation_matrix_z, temp_coordinates)
			coordinates = np.dot(rotation_matrix_x, coordinates)
			x = coordinates[0]; y = coordinates[1]; z = coordinates[2]
			if( ( (z+0.1)>0) and (x>1)):
				mapa[int(x*100+5000)][int(y*100+1500)] = 255
				x_global = orientation*x*100 + gps_x*100 + orientation*23.27
				y_global = orientation*y*100 + gps_y*100 + 0.002*100
				if( (int(y_global+1500)<3000)):
					mapa_global[int(x_global+6000)][int(y_global+1500)] = 255
		cv2.imwrite('mapa1.png', mapa)
		mapa_global = cv2.morphologyEx(mapa_global, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
		cv2.imwrite('mapaglobal.png', mapa_global)
def listener():
	rospy.init_node('Map_Generator', anonymous=True)
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/sensor/imu', Imu, IMU)
	rospy.Subscriber('/sensor/velodyne', PointCloud2, VELODYNE)
	#talvez colocar um sleep resolva
	rospy.spin()
	
if __name__ == "__main__":
	try:	
		listener()
	except rospy.ROSInterruptException:
		print ("ROS Interupt Exception")
