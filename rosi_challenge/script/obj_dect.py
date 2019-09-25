#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image
from PIL import Image as IMG
from sensor_msgs.msg import NavSatFix
import cv2
import math
gps_x_global, gps_y_global, gps_z_global = 0,0,0
inn=0
def GPS(GPS_data):
	global gps_x_global
	global gps_y_global
	global gps_z_global
	global inn
	gps_x_global = GPS_data.latitude
	gps_y_global = GPS_data.longitude
	gps_z_global = GPS_data.altitude
	inn += 1
	
def callback(datas):
	global inn
	print inn
	if(inn>=6):
		inn=0
		dist = np.empty((480, 640), dtype = np.uint16)
		imagem = np.empty((480, 640), dtype = np.uint8)
		k = 0
		valor_medio = 0
		dist_media = 0
		dist_esc = 0
		total = 0
		begin = 0
		void = 0
		is_void = False
		stair = False
		initial_h = 0
		valor_medio_e = 0
		desv = 0
		 
		for i in range(480):
			for j in range(640):
				dist[i, j] = ord(datas.data[2 * k]) + (ord(datas.data[2*k + 1]) << 8)
				imagem[i, j] = dist[i, j] // 13.6718751
				k += 1
	    
		for k in range(480):
		   if imagem[k][10] != 255:
		        k = k - 10
		        break

		for l in range(380):
		    if imagem[k - 10][l] != 255 and begin == 0:
		    	begin = l
		    if imagem[k - 10][l] == 255 and begin != 0 and void == 0:
		    	void = l + 10
		    	is_void = True
		    	break
		    	
		if begin != 0:
			if is_void:
				for p in range(k, 100, -1):
					if imagem[p][void] != 255:
						is_void = False
						break
				    
			for m in range (k-10, k):
			    for n in range(begin, begin+10):
				total += 1
				valor_medio += imagem[m][n + 30]
				dist_media += dist[m][n + 30]
				dist_esc += dist[m][n]
				imagem [m][n] = 0
			if valor_medio/total > 240:
				for o in range(k, 480):
				        if imagem[o][begin] != 255:
				                stair = True
				                break
				if stair:
				 	for q in range(begin + 50, 640):
						#rospy.loginfo("%d", q)
				 		if imagem[k][q]!=255:
				 			initial_h = q
				 			break
				 	#rospy.loginfo("%d, %d", initial_h)		
				 	for m in range (k-10, k):
				      		for r in range(initial_h, initial_h+10):
							valor_medio_e += dist[m][r]
							imagem [m][r] = 0
					rospy.loginfo("Stair.")
					desv = (((begin-320)*0.00173667*(dist_esc/total)+500) - ((initial_h-320)*0.00173667* (valor_medio_e/total)-500))/2
					if desv < 0:
						rospy.loginfo("X: %d, desviar para esquerda %d", dist_esc/total, desv)
					elif desv > 0:
						rospy.loginfo("X: %d, desviar para direita %d", dist_esc/total, desv)
					#rospy.loginfo("X: %d Y esquerda: %d Y direita: %d", dist_esc/total, (begin-320)*0.00173667* (dist_esc/total)+500, (initial_h-320)*0.00173667* (valor_medio_e/total)-500)
			else:
				if is_void:
					rospy.loginfo("Object with void.")
					rospy.loginfo("X: %d Y: %d", dist_media/total, (begin-320)*0.00173667* (dist_esc/total)+800)
				else:
					rospy.loginfo("X: %d, Y: %d", dist_media / total, (begin-320)*0.00173667* (dist_esc/total)-500)
			
		cv2.imshow("Image", imagem)
		if cv2.waitKey(1) & 0xFF == ord('q'):
	    		cv2.destroyAllWindows()
			rospy.signal_shutdown('FIM')

def listener():
	rospy.init_node('SENSOR1')
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/sensor/kinect_depth', Image, callback)
	rospy.spin()
	
if __name__ == "__main__":
	try:	
		listener()
	except rospy.ROSInterruptException:
		print "eeeeee"
