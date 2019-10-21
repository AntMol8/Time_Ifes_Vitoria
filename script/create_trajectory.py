#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

object_x = 0
object_y = 0
core_flag = 0           #used to accept or not new values
stop_flag = 0
orientation = -1
speed = 2
gps_x_ref, gps_y_ref = 0, 0
gps_x, gps_y = 0, 0
front_angle = 0
back_angle = 0

def Main_Control_Parameters(data):
	#receives information from its subscriber
	global gps_x_ref, gps_y_ref
	global gps_x, gps_y, speed, front_angle, back_angle
	global object_x, object_y, core_flag

	core_flag = data.data[0]
	stop_flag = data.data[1]
	object_x = data.data[2]
	object_y = data.data[3]
	speed = data.data[4]
	front_angle = data.data[5]
	back_angle = data.data[6]
        
	gps_x_ref = gps_x
	gps_y_ref = gps_y
	
	slave_msg = rospy.Publisher('/create_trajectory_feedback', Float32MultiArray, queue_size = 1)
	slave_publication = Float32MultiArray()
	slave_publication.data = (1, 0)
	slave_msg.publish(slave_publication)
	
def GPS(data):
	#uses the robots coordinates to determine the parameters of the hyberbolic tangent
	global object_x, object_y, speed, front_angle, back_angle
	global gps_x_ref, gps_y_ref
	global orientation, core_flag
	global gps_x, gps_y
	
	gps_x = data.latitude
	gps_y = data.longitude
	delta_x = gps_x - object_x #exchanges gps_x for gps_x_ref
	delta_y = -gps_y + object_y #exchanges gps_y for gps_y_ref

	if (abs(delta_x) > 0.3):
		#b is a constant used in the hyberbolic tangent
		b = 2.646652412 / (delta_x/np.absolute(delta_x)*np.absolute(abs(delta_x)-0.3)/2)
	else:
		b = 2.646652412 / (delta_x/np.absolute(delta_x)*np.absolute(abs(delta_x))/2)
        
	if (delta_x < 0):
		orientation = 1
	else:
		orientation = -1
                
	amplitude = delta_y / 2
	resolution = orientation * 0.15
	msg = rospy.Publisher('/create_trajetory', Float32MultiArray, queue_size = 10)
	pub = Float32MultiArray()
	pub.data = (stop_flag, resolution, amplitude, b, speed, front_angle, back_angle, gps_x_ref, gps_y_ref)

	if (core_flag != 0):
		msg.publish(pub)
        
	core_flag = 0

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
