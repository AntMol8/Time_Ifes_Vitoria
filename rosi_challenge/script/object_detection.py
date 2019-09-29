#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
import cv2
gps_x_global, gps_y_global, gps_z_global = 0.0,0.0,0.0
dest_x_global, dest_y_global = 0.0, 0.0
inn=0
total_g = 0

# Recives the GPS coordinates
def GPS(GPS_data):
	global gps_x_global
	global gps_y_global
	global gps_z_global
	global inn
	gps_x_global = GPS_data.latitude
	gps_y_global = GPS_data.longitude
	gps_z_global = GPS_data.altitude
	inn += 1
	
# Object detection
def callback(datas):
	global inn
	global dest_x_global
	global dest_y_global
	global total_g
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
		
		# Matrix of the kinect image
		for i in range(480):
			for j in range(640):
				dist[i, j] = ord(datas.data[2 * k]) + (ord(datas.data[2*k + 1]) << 8)
				imagem[i, j] = dist[i, j] // 13.6718751
				k += 1
			
		# Method to find one end of the obstacle
		if imagem[0][0] == 255 and imagem[0][630] != 255:
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
			    	
			# Identification of the restrict area
			if begin != 0:
				if is_void:
					for p in range(k, 100, -1):
						if imagem[p][void] != 255:
							is_void = False
							break
					    
				# Calculate the the average distance of the region of pixels
				for m in range (k-10, k):
				    for n in range(begin, begin+10):
					total += 1
					valor_medio += imagem[m][n + 30]
					dist_media += dist[m][n + 30]
					dist_esc += dist[m][n]

				# Indentification of the stair
				if valor_medio/total > 240:
					for o in range(k, 480):
						if imagem[o][begin] != 255:
						        stair = True
						        break
					if stair:
					 	for q in range(begin + 50, 640):
					 		if imagem[k][q]!=255:
					 			initial_h = q
					 			break
					 	for m in range (k-10, k):
					      		for r in range(initial_h, initial_h+10):
								valor_medio_e += dist[m][r]
								#imagem [m][r] = 0
						
						desv = (((320-begin)*0.00173667*(dist_esc/total)) + ((320-initial_h)*0.00173667* (valor_medio_e/total)))/2
						# Calculates the destination spot
						dest_x = -dist_esc/(total*1000) + gps_x_global +1
						dest_y = desv/1000+ gps_y_global 
						if dist_esc/total < 3000 and dist_esc/total > 2800:
							dest_x_global += dest_x
							dest_y_global += dest_y
							total_g +=1
						elif total_g != 0 and dist_esc/total < 2800:
							dest_x_global = dest_x_global/total_g
							dest_y_global = dest_y_global/total_g
							rospy.loginfo("Stair.")
							rospy.loginfo("X: %f Y: %f", dest_x_global, dest_y_global)
							total_g = 0
							dest_x = 0
							dest_y = 0
				else:
					if is_void:
					        
					    # Calculates the destination spot
						dest_x = dist_media/(total*1000) + gps_x_global
						dest_y = (begin-320)*0.00173667* (dist_esc/(total*1000))+0.65+ gps_y_global
						if dist_media/total < 2300 and dist_media/total > 1850:
							dest_x_global += dest_x
							dest_y_global += dest_y
							total_g +=1
						elif total_g != 0 and dist_media/total < 1850:
							dest_x_global = dest_x_global/total_g
							dest_y_global = dest_y_global/total_g
							rospy.loginfo("Object with void.")
							rospy.loginfo("X: %f Y: %f", dest_x_global, dest_y_global)
							total_g = 0
							dest_x = 0
							dest_y = 0
					else:
						# Calculates the destination spot
						dest_x = dist_media/(total*1000) + gps_x_global
						dest_y = (begin-320)*0.00173667* (dist_esc/(total*1000))-0.5 + gps_y_global
						if dist_media/total < 2500 and dist_media/total > 2300:
							dest_x_global += dest_x
							dest_y_global += dest_y
							total_g +=1
						elif total_g != 0 and dist_media/total < 2300:
							dest_x_global = dest_x_global/total_g
							dest_y_global = dest_y_global/total_g
							if abs(dest_y_global) >= 4:
								dest_y_global = 3.5
							rospy.loginfo("Obstacle.")	
							rospy.loginfo("X: %f Y: %f", dest_x_global, dest_y_global)
							total_g = 0
							dest_x = 0
							dest_y = 0
							
		if cv2.waitKey(1) & 0xFF == ord('q'):
	    		cv2.destroyAllWindows()
			rospy.signal_shutdown('FIM')

def listener():
	rospy.init_node('object_detection')
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/sensor/kinect_depth', Image, callback)
	rospy.spin()
	
if __name__ == "__main__":
	try:	
		listener()
	except rospy.ROSInterruptException:
		pass