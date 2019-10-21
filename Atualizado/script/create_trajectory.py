#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

x_objeto = 0
y_objeto = 0
flag_core = 0
flag_parar = 0
orientation = -1
speed = 2
gps_x_ref, gps_y_ref = 0, 0
gps_x, gps_y = 0, 0
front_angle = 0
back_angle = 0

def Main_Control_Parameters(data):
	global gps_x_ref, gps_y_ref
	global gps_x, gps_y, speed, front_angle, back_angle
	global x_objeto, y_objeto, flag_core #flag_core serve para aceitar ou nao os novos valores
	flag_core = data.data[0]
	flag_parar = data.data[1]
        x_objeto = data.data[2]
        y_objeto = data.data[3]
        speed = data.data[4]
        front_angle = data.data[5]
        back_angle = data.data[6]
        
        gps_x_ref = gps_x
        gps_y_ref = gps_y
        msg_l = rospy.Publisher('/create_trajectory_feedback', Float32MultiArray, queue_size = 1)
        pub_l = Float32MultiArray()
        print 'create_trajectory: recebeu do main_control'
	print 'create_trajectory: x: ', data.data[2], 'y: ', data.data[3]
        a_l = (1,0)
        pub_l.data = a_l
        msg_l.publish(pub_l)
	
def GPS(data): #para o calculo da orientacao tem que saber a orientacao
	global x_objeto, y_objeto, speed, front_angle, back_angle
	global gps_x_ref, gps_y_ref
	global orientation, flag_core
	global gps_x, gps_y
	
	gps_x = data.latitude
	gps_y = data.longitude
        delta_x = gps_x - x_objeto #substitui gps_x por gps_x_ref
        if(abs(delta_x)>0.3):
        	b = 2.646652412 / (delta_x/np.absolute(delta_x)*np.absolute(abs(delta_x)-0.3)/2) #0.3 eh para dar uma margem de seguranca
        else:
        	b = 2.646652412 / (delta_x/np.absolute(delta_x)*np.absolute(abs(delta_x))/2)
        delta_y = -gps_y + y_objeto #substitui gps_y por gps_y_ref
        
        if (delta_x < 0):
                orientation = 1 #manda no canal de comunicacao com o mestre
        else:
                orientation = -1
                
        amplitude = delta_y / 2
        resolucao = orientation * 0.15
        msg = rospy.Publisher('/create_trajectory', Float32MultiArray, queue_size = 10)
        pub = Float32MultiArray()
        a = (flag_parar, resolucao, amplitude, b, speed, front_angle, back_angle, gps_x_ref, gps_y_ref)
        pub.data = a
        if(flag_core!=0):
        	print 'create_trajectory: delta_y trajectory_parameters', amplitude
		msg.publish(pub)
	flag_core=0

def talker():
	rospy.init_node('Hyperbolic_Tangent', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/main_control', Float32MultiArray, Main_Control_Parameters)	
	rospy.spin()
	
if __name__ == '__main__':
	try:
	    talker()
	except rospy.ROSInterruptException:
	    pass