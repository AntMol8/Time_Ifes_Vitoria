#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
import math

resolution = -0.15 #10 cm
b = 2.0				#b is a constant used in the hyberbolic tangent
constant = 2.646652412 / b #constantes de natureza empirica baseada no valor da distancia do objeto
amplitude = 0
x_ref, y_ref = -5, -2
stop_flag = 0
speed = 1.5
front_angle = 0
back_angle = 0
gps_x_ref, gps_y_ref = 0, -2

def Trajectory_Parameters(data):
	global speed, front_angle, back_angle
	global amplitude
	global b, constant
	global stop_flag
	global gps_x_ref, gps_y_ref
	global x_ref, y_ref, resolution

	if (gps_x_ref != data.data[7]): #-------acho que precisa de mais coisa
		stop_flag = data.data[0]
		resolution = data.data[1]
		amplitude = data.data[2] 
		b = data.data[3]
		speed = data.data[4]
		front_angle = data.data[5]
		back_angle = data.data[6]	
		gps_x_ref = data.data[7]
		gps_y_ref = data.data[8]

		constant = 2.646652412 / b
		x_ref = gps_x_ref + resolution
		arg = -b * (x_ref+constant-gps_x_ref)
		y_ref = math.tanh(arg) + 0
        y_ref = amplitude * (y_ref+1)
        y_ref += gps_y_ref
        print 'trajectory_parameters: recebeu do create_trajectory'

def GPS(data):
	global resolution
	global constant, amplitude
	global b
	global stop_flag
	global x_ref, y_ref
	global speed, front_angle, back_angle
	global gps_x_ref, gps_y_ref

	gps_x = data.latitude
	gps_y = data.longitude

    if (stop_flag == 1):
        speed = 0
    elif (((np.absolute(gps_x - x_ref) < 0.05) and (np.absolute(gps_y - y_ref) < 0.10)) or (np.absolute(gps_x - x_ref) < 0.02)):
        #f(x) = a * tgh(-b (x+(2.6466)/(b)-d)) + a + c
        x_ref = gps_x + resolution
        arg = -b * (x_ref+constant-gps_x_ref)
        y_ref = math.tanh(arg) + 0
        y_ref = amplitude * (y_ref + 1)
        y_ref += gps_y_ref
        print "Mudou", "amplitude ", amplitude, 'b: ', b, 'constant: ', constant, 'gps_y_ref: ', gps_y_ref, 'gps_x_ref: ', gps_x_ref, 'y_ref: ', y_ref, 'x_ref: ', x_ref
        
	msg = rospy.Publisher('/trajectory_parameters', Float32MultiArray, queue_size = 1)
	pub = Float32MultiArray()
	pub.data = (x_ref, y_ref, speed, front_angle, back_angle)
	msg.publish(pub)

def talker():
	rospy.init_node('Path_Planning', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/create_trajetory', Float32MultiArray, Trajectory_Parameters)
	rospy.spin()
	
if __name__ == '__main__':
	try:
	    talker()
	except rospy.ROSInterruptException:
	    pass