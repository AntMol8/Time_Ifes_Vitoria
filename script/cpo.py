#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Int8
from sensor_msgs.msg import NavSatFix
gps_x, gps_y = 0, 0
x_velodyne, y_velodyne = 0, 0
x_kinect, y_kinect = 0, 0
objeto = 0
flag_retorno = 0
flag_volta = 0
flag_desvio = 0 #0 eh false
x_desvio_ref = 0
y_desvio_ref = 0
def RETORNO(data):
        global flag_retorno

	flag_retorno = data.data[0]

def Velodyne(data):
	global x_velodyne, y_velodyne
	x = data.data[0]
	y = data.data[1]
	x_velodyne = x
	y_velodyne = y
	
def Kinect(data):     
	global gps_x, gps_y
	global x_kinect, y_kinect, objeto
	objeto = data.data[0]
	x = data.data[1]
	y = data.data[2]
	if (abs(gps_y) - 1.85) < 0.15:
		x_kinect = x
		y_kinect = y
	
def GPS(data):
	global gps_x, gps_y, objeto
	global x_kinect, y_kinect, x_desvio_ref, y_desvio_ref
	global x_velodyne, y_velodyne
	global flag_retorno, flag_desvio, flag_volta
        gps_x = data.latitude
	gps_y = data.longitude
	
	
	
	
	
	if(flag_desvio==0 and flag_volta == 0):
		if (x_velodyne != 0 and y_velodyne != 0 and x_kinect != 0 and y_kinect != 0):
			flag_desvio = 1
			erro_x = x_velodyne - x_kinect
			erro_y = y_velodyne - y_kinect
			print 'erro_x: ', erro_x
			print 'erro_y: ', erro_y
			pub_main = rospy.Publisher('/canaldocpo', Float32MultiArray, queue_size = 1)
			a = Float32MultiArray()
			
			if(objeto==1):
				y_pub = (y_kinect)
				#analisar qual dos dois valores pegar se erro for muito grande
				x_pub = (x_kinect)
			else:
				y_pub = (y_kinect + y_velodyne)/2
				#analisar qual dos dois valores pegar se erro for muito grande
				x_pub = (x_kinect + x_velodyne)/2
				
			a.data = (objeto, x_pub, y_pub, 2, 180, 0, 0,1)
			x_desvio_ref = x_pub
			y_desvio_ref = y_pub
			while (flag_retorno != 1):
		        	print "-"
			        pub_main.publish(a)
			flag_volta = 0
			flag_retorno = 0
			x_kinect = 0
			y_kinect = 0
			x_velodyne = 0
			y_velodyne = 0
	elif(flag_desvio==1 and flag_volta == 0):
		if( np.absolute(x_desvio_ref - gps_x) < 0.05 and (y_desvio_ref - gps_y) < 0.05):
			flag_desvio = 0
			flag_volta = 1
			pub_main = rospy.Publisher('/canaldocpo', Float32MultiArray, queue_size = 1)
			a = Float32MultiArray()
			a.data[8]=2
			x_velodyne = 0 #?????
			
			
			
	elif(flag_desvio==0 and flag_volta == 1):	
		if (np.absolute(x_velodyne - gps_x) < 0.05):
			flag_volta = 0
			pub_main = rospy.Publisher('/canaldocpo', Float32MultiArray, queue_size = 1)
			a = Float32MultiArray()
			if(gps_y>0):
				y_pub = 1.85
				x_pub = x_gps-1.3
				
			else:
				y_pub = -1.95
				x_pub = x_gps+1.3
			#analisar qual dos dois valores pegar se erro for muito grande
			a.data = (objeto, x_pub, y_pub, 2, 180, 0, 0,1)
			while (flag_retorno != 1):
		        	print "-"
			        pub_main.publish(a)
			flag_retorno = 0
			x_kinect = 0
			y_kinect = 0
		
		
def talker():
	rospy.init_node('OBJETOS', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/canaldovelodyne', Float32MultiArray, Velodyne)
	rospy.Subscriber('/canaldokinect', Float32MultiArray, Kinect)
	rospy.Subscriber('/canaldocpmo', Float32MultiArray, RETORNO)
	rospy.spin()
	
if __name__ == '__main__':
	try:
	    talker()
	except rospy.ROSInterruptException:
	    pass
