#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
import math
resolucao = -0.15 #10 cm
flag_comeco = 1
b = 2.0
desc = 2.646652412/b #constantes de natureza empirica baseada no valor da distancia do objeto
amplitude = 0
x_ref=-5; y_ref=-2
flag_parar = 0
speed = 1.5
front_angle = 0
back_angle = 0
gps_x_ref=0; gps_y_ref=-2

def Trajectory_Parameters(data):
	global direita #concavidade da curva tanh
	global flag_objeto, speed, front_angle, back_angle
	global amplitude
	global b, desc
	global flag_parar
	global gps_x_ref, gps_y_ref
	global x_ref, y_ref, resolucao
	if(gps_x_ref!=data.data[7]): #-------acho que precisa de mais coisa
		flag_parar = data.data[0]
		resolucao = data.data[1]
		amplitude = data.data[2] 
		b = data.data[3]
		speed = data.data[4]
		front_angle = data.data[5]
		back_angle = data.data[6]
		desc = 2.646652412/b
		gps_x_ref = data.data[7]
		gps_y_ref = data.data[8]
		x_ref = gps_x_ref+resolucao
		arg = -b*(x_ref+desc-gps_x_ref)
		y_ref = math.tanh(arg) + 0
                y_ref = y_ref*amplitude + amplitude 
                y_ref = y_ref + gps_y_ref
                print 'trajectory_parameters: recebeu do create_trajectory', x_ref, y_ref
def GPS(data):
	global resolucao
	global desc, amplitude
	global b
	global flag_parar
	global flag_objeto
	global x_ref, y_ref
	global speed, front_angle, back_angle
	global gps_x_ref, gps_y_ref
	gps_x = data.latitude
	gps_y = data.longitude
        if(flag_parar==1):
                speed = 0
        elif ( ((np.absolute(gps_x - x_ref) < 0.05) and (np.absolute(gps_y - y_ref) < 0.10)) or (np.absolute(gps_x - x_ref) < 0.02) ):
                errox = gps_x - x_ref; erroy = gps_y - y_ref
                #f(x)=a*tgh(-b (x+(2.6466)/(b)-d))+a+c
                x_ref = gps_x + resolucao
                arg = -b*(x_ref+desc- gps_x_ref)
                #arg = arg  #mudei o sinal da referencia
                y_ref = math.tanh(arg) + 0
                y_ref = y_ref*amplitude + amplitude #o sinal do antigo flag direita ja esta incluso na amplitude
                y_ref = y_ref + gps_y_ref
                print "trajectory_parameters: Mudou", "amplitude ", amplitude, 'b: ', b, 'desc: ', desc, 'gps_y_ref: ', gps_y_ref, 'gps_x_ref: ', gps_x_ref, 'y_ref: ', y_ref, 'x_ref: ', x_ref
                #com 2 m/s, 3 minutos simulados para chegar na escada
	msg = rospy.Publisher('/trajectory_parameters', Float32MultiArray, queue_size = 1)
	pub = Float32MultiArray()
	a = (x_ref, y_ref, speed, front_angle, back_angle)
	pub.data = a
	msg.publish(pub)

def talker():
	rospy.init_node('Path_Planning', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/create_trajectory', Float32MultiArray, Trajectory_Parameters)
	rospy.spin()
	
if __name__ == '__main__':
	try:
	    talker()
	except rospy.ROSInterruptException:
	    pass
