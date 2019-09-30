#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math

def listener():
	rospy.init_node('ORIENTATION_KINECT', anonymous = True)
	rospy.Subscriber('/sensor/imu', Imu, callback)
	rospy.spin()
	
def callback(data):
	angle = rospy.Publisher('/rosi/command_kinect_joint', Float32, queue_size=10)
	
	X = float(data.orientation.x)
	Y = float(data.orientation.y)
	Z = float(data.orientation.z)
	W = float(data.orientation.w)
	
	quat_to_radians1= (np.arctan(2*(X*Y + Z*W)))/(1-2*(Y**2+Z**2))
	quat_to_radians2 = np.arcsin(2*(X*Z - W*Y))
	
	if(quat_to_radians2 > math.pi/4):
		quat_to_radians2 = -math.pi/4
		
	if(quat_to_radians2 < -math.pi/4):
		quat_to_radians2 = math.pi/4 
		
	#print(quat_to_radians2*-1)
	
	angle.publish(quat_to_radians2*-1)
	
if __name__ == "__main__":
	try:	
		listener()
	except rospy.ROSInterruptException:
		print "Exception occured. Shutting down."
