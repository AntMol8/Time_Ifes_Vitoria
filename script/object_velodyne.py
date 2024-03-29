#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
import cv2
import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from struct import unpack, pack

imu_x_global, imu_y_global, imu_z_global = 0, 0, 0
gps_x_global, gps_y_global, gps_z_global = 0.0, 0.0, 0.0
synchronizer = 50
flag = 1
global_flag = 1

def Flag_Velodyne(data):		#receives a flag from object_control to know what to identify
    global global_flag

    msg = rospy.Publisher('/velodyne_feedback', Float32MultiArray, queue_size = 1)
    pub = Float32MultiArray()
    pub.data = (1, 0)
    msg.publish(pub)
    global_flag = data.data[0]

def GPS(GPS_data):			#gets GPS data
	global gps_x_global
	global gps_y_global
	global gps_z_global
	global synchronizer
	
	synchronizer += 1
	
	gps_x_global = GPS_data.latitude
	gps_y_global = GPS_data.longitude
	gps_z_global = GPS_data.altitude

def IMU(Imu_data):			#gets IMU data
	global imu_x_global
	global imu_y_global
	global imu_z_global
	global orientation
	global synchronizer
	
	synchronizer += 1
	
	a = Imu_data.orientation.w
	b = Imu_data.orientation.x
	c = Imu_data.orientation.y
	d = Imu_data.orientation.z
	ref = math.sqrt((1 - b**2 - c**2) / 2)

	imu_x_global = np.arctan((2*(a*b + c*d)) / (a*a - b*b - c*c + d*d))
	imu_y_global = np.arcsin(2 * (b*d - a*c))
	imu_z_global = np.arctan((2*(a*d + c*b)) / (a*a + b*b - c*c - d*d))

def Velodyne(dado):			#detects objects using velodyne and publishes to object_control
	global imu_x_global
	global imu_y_global
	global imu_z_global
	global gps_x_global
	global gps_y_global
	global gps_z_global
	global orientation
	global global_flag
	global synchronizer
	
	if (synchronizer > 50):			#synchronize the velodyne with the IMU and the GPS
		synchronizer = 0
		gps_x = gps_x_global
		gps_y = gps_y_global
		
		flag = global_flag		#saves the flag state to avoid changes during execution 
		
		mapa = np.zeros((10000, 3000), np.uint8)

		imu_x = imu_x_global
		imu_y = imu_y_global
		imu_z = imu_z_global
		angulo_y = np.pi/12 - imu_y
		obj_y = 1
		obj_x = 0
		
		msg = rospy.Publisher('/object_velodyne', Float32MultiArray, queue_size = 1)
		pub = Float32MultiArray()

		for i in range(0, len(dado.data), 12):
			value_x = unpack('f', dado.data[i: i+4])[0]
			value_z = unpack('f', dado.data[i+8: i+12])[0]		#unpack velodyne data to use in code
			value_y = unpack('f', dado.data[i+4: i+8])[0]
			
			#rotates velodyne plane to line up with the world plane
			ang = np.array([[value_x], [value_y], [value_z]])
			rotz = np.array([[np.cos(imu_z), -np.sin(imu_z), 0], [np.sin(imu_z), np.cos(imu_z), 0], [0, 0, 1]])
			rotx = np.array([[1, 0, 0], [0, np.cos(imu_x), -np.sin(imu_x) ], [0, np.sin(imu_x), np.cos(imu_x)]])
			roty = np.array([[np.cos(angulo_y), 0, np.sin(angulo_y)], [0, 1, 0], [-np.sin(angulo_y), 0, np.cos(angulo_y)]])
			ang_y = np.dot(roty,ang)
			coordinates = np.dot(rotz, ang_y)
			coordinates = np.dot(rotx, coordinates)
			x = coordinates[0]; y = coordinates[1]; z = coordinates[2]
			
			if flag == 1:
				if (z >= -0.25) and (1 <= x <= 3.5) and (-1.25 <= y <= 0.25):	#detects object beginning
					mapa[int(x*100 + 5000), int(y*100 + 1500)] = 255
					if y < obj_y:
						obj_y = y
						obj_x = x
				
			elif flag == 2:
				if (z >= -0.25) and (0 <= x <= 3) and (1 > y > 0.3):	#detects object end
					mapa[int(x*100 + 5000), int(y*100 + 1500)] = 255
					if x > obj_x:
						obj_x = x
						obj_y = y
		
		if obj_x != 0:					#corrects the difference between gps and velodyne reference
			if gps_y_global	> 0:
				obj_x = gps_x - obj_x - 0.2425
				obj_y = gps_y - obj_y
				if flag == 1:
					obj_y += 0.65
			else:
				obj_x += gps_x + 0.2425
				obj_y += gps_y
				if flag == 1:
					obj_y -= 0.65
			
			
			pub.data = (obj_x, obj_y)
			msg.publish(pub)		
			
def listener():
	rospy.init_node('SENSOR12', anonymous = True)
	rospy.Subscriber('/sensor/imu', Imu, IMU)
	rospy.Subscriber('/sensor/velodyne', PointCloud2, Velodyne)
	rospy.Subscriber('/OBC_OBV', Float32MultiArray, Flag_Velodyne)
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.spin()
	
if __name__ == "__main__":
	try:	
		listener()
	except rospy.ROSInterruptException:
		print ("ROS Interupt Exception")
