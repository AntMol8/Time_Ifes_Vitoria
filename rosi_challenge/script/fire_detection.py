#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv2

aux = 0
lower = np.array([0, 50, 150])
upper = np.array([100, 255, 255])

def callback(datas):
	global aux, lower, upper
	fires = []
	
	img = cv2.cvtColor(np.fromiter(map(ord, datas.data), dtype=np.uint8).reshape(480, 640, 3), cv2.COLOR_RGB2BGR)[0:240, :, :]
	binary = cv2.inRange(img, lower, upper)
	mask = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 50)))
	
	cv2.imshow('binary', binary)
	cv2.waitKey(1)
	
	cv2.imshow('mask', mask)
	cv2.waitKey(1)
	
	cnt = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
	
	if len(cnt) < aux:
		print("FOGO")
		
	aux = len(cnt)

	for i in cnt:
		x, y, w, h = cv2.boundingRect(i)
		px, py = x + (w // 2), y + ((4 * h) // 5)
		img[py-1: py+1, px-1: px+1] = [0, 255, 0]
		fires.append((px,py))
		cv2.rectangle(img, (x, y),(x + w, y + h),(0, 255, 0), 2)

	cv2.imshow('img', img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		cv2.destroyAllWindows()
		rospy.signal_shutdown('FIM')

def listener():
	rospy.init_node('FireKinectRgb', anonymous = True)
	rospy.Subscriber('/sensor/kinect_rgb', Image, callback, queue_size = 1)
	rospy.spin()
	
if __name__ == "__main__":
	try:	
		listener()
	except rospy.ROSInterruptException:
		print "ROS Interupt Exeption"
