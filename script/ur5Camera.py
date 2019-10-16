#!/usr/bin/env python
import numpy as np
import rospy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from rosi_defy.msg import HokuyoReading
from std_msgs.msg import Float32MultiArray
import cv2
#Variable declaration
data_x = 10
data_y = 10
pos_x, pos_y, pos_z, x, y = 0.0, 0.0, 0.0, 0.0, 0.0
memory=()
fire = 0
previous_fires_x = []
previous_fires_y = []
	
lower = np.array([0, 150, 150])
upper = np.array([50, 255, 255])

def position(GPS_data):
	global previous_fires_x
	global previous_fires_y
	global fire
	#Receives coordinates x and y from the gps
	global pos_x
	global pos_y
	pos_x = GPS_data.latitude
	pos_y = GPS_data.longitude
	check = 0
	if(fire == 1):
	        if(pos_y > 1):
	                pos_y = 1.245
	        elif (pos_y < -1):
	        	pos_y = -1.245
	       	else:
	       		print("error")
	       		return
	       	
		for i in range(len(previous_fires_x)):
			if (pos_x + 0.2 > previous_fires_x[i] and pos_x - 0.2 < previous_fires_x[i]) and (pos_y + 0.2 > previous_fires_y[i] and pos_y - 0.2 < previous_fires_y[i]):
				check = 1
		if check == 0:
			pub.data = (pos_x,pos_y)
			print(pub)
			flag.publish(pub)
			previous_fires_x.append(pos_x)
			previous_fires_y.append(pos_y)
		fire = 0

def callback_u(datas):
	global aux, lower, upper
	global fire
	img = cv2.cvtColor(np.fromiter(map(ord, datas.data), dtype=np.uint8).reshape(480, 640, 3)[:, 410:430, :], cv2.COLOR_RGB2BGR)
	binary = cv2.inRange(img, lower, upper)
	mask = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 50)))
	
	cv2.imshow('binary', binary)
	cv2.waitKey(1)
	
	cv2.imshow('mask', mask)
	cv2.waitKey(1)
	
	cnt = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
	
	if len(cnt) > 0:
		print("FOGO")
		fire = 1

	for i in cnt:
		x, y, w, h = cv2.boundingRect(i)
		px, py = x + (w // 2), y + ((4 * h) // 5)
		img[py-1: py+1, px-1: px+1] = [0, 255, 0]
		cv2.rectangle(img, (x, y),(x + w, y + h),(0, 255, 0), 2)

	cv2.imshow('img', img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		cv2.destroyAllWindows()
		rospy.signal_shutdown('FIM')

def listener():
#Inits node and declares subscribers
	global pub
	global flag
	rospy.init_node('cameraUR5', anonymous = True)
	rospy.Subscriber('/sensor/ur5toolCam', Image, callback_u, queue_size = 1)
	rospy.Subscriber('/sensor/gps', NavSatFix, position)
	flag = rospy.Publisher('/detect_fire', Float32MultiArray, queue_size=1)
	pub = Float32MultiArray()
	pub.data=0
	rospy.spin()
	
if __name__ == "__main__":
	try:	
		listener()
	except rospy.ROSInterruptException:
		print "ROS Interupt Exeption"