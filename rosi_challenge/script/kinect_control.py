#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32
import cv2
import math

def falador():
		
		#ang=float(input('angulo '))
		#inp=(0.1*ang)/10.2
		#print(inp)
	while not rospy.is_shutdown():	
		ang = input('angulo? ')
		if (ang>45):
			ang = 45
		if (ang<-45):
			ang = -45
		out = (ang*math.pi)/180
		print 'angulo', ang, 'saida', out
		
		pub = rospy.Publisher('/rosi/command_kinect_joint', Float32, queue_size=1)
		rospy.init_node('KinectJointControl')
		pub.publish(out)

		


if __name__ == "__main__":
	try:	
		falador()
	except rospy.ROSInterruptException:
		print "eeeeee"
