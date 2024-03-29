#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Int8
from sensor_msgs.msg import NavSatFix
gps_x, gps_y = 0, 0
x_velodyne, y_velodyne = 0, 0
x_kinect, y_kinect = 0, 0
objeto = 2
flag_retorno = 0
flag_velodyne = 0
flag_volta = 0
flag_desvio = 0 #0 eh false
x_desvio_ref = 0
y_desvio_ref = 0
temp_x_ref = 0
temp_y_ref = 0

def Velodyne_Feedback(data): #Feedback from velodyne about path alteration
        global flag_velodyne
	
	flag_velodyne = data.data[0]

def MC_Feedback(data): # feedback from main control about path alteration
        global flag_retorno

	flag_retorno = data.data[0]

def Velodyne(data): #gets data from velodyne to initiate the change in course to avoid obstacles
	global x_velodyne, y_velodyne, gps_x, gps_y, flag_desvio, flag_volta, objeto, x_kinect
	x = data.data[0]
	y = data.data[1]
	print 'velodyne'
	if(flag_desvio==0 and flag_volta == 0 and objeto != 1 and x_kinect != 0):
		x_velodyne = x
		y_velodyne = y
	else:
		x_velodyne = 0
		y_velodyne = 0
	
def Kinect(data):  #gets data from kinect to initiate the change in course to avoid obstacles
	global gps_x, gps_y, flag_desvio, flag_volta
	global x_kinect, y_kinect, objeto
	objeto = data.data[0]
	x = data.data[1]
	y = data.data[2]
	print 'kinect'
	if (abs(gps_y) - 1.85) < 0.2:
		if(flag_desvio==0 and flag_volta == 0):
			x_kinect = x
			y_kinect = y
			objeto = data.data[0]
		else:
			x_kinect = 0
			y_kinect = 0
			objeto = data.data[0]
	
def GPS(data):
	global gps_x, gps_y, objeto
	global x_kinect, y_kinect, x_desvio_ref, y_desvio_ref
	global x_velodyne, y_velodyne
	global flag_retorno, flag_desvio, flag_volta
	global flag_velodyne
	global temp_x_ref, temp_y_ref
        gps_x = data.latitude
	gps_y = data.longitude
	print 'cpo: gps', objeto,"desvio: ", flag_desvio, "volta: ", flag_volta, "x: ", gps_x
	print 'x_velodyne: ', x_velodyne, 'x_kinect: ', x_kinect
	if((-6.31>gps_x>-41.0)):
		if(objeto!=1):
			if(flag_desvio==0 and flag_volta == 0):
				msg_l = rospy.Publisher('/OBC_OBV', Float32MultiArray, queue_size = 1) 
				while (flag_velodyne != 1):
					pub_l = Float32MultiArray()
					print 'cpo: controlando o flag_v'
					a_l = (1,0)
					pub_l.data = a_l
					msg_l.publish(pub_l)
					x_velodyne = 0
					y_velodyne = 0
					x_kinect = 0
					y_kinect = 0
					flag_velodyne = 0
				
				if (x_velodyne != 0 and y_velodyne != 0 and x_kinect != 0 and y_kinect != 0):
					erro_x = x_velodyne - x_kinect
					erro_y = y_velodyne - y_kinect
					print 'cpo: erro_x: ', erro_x
					print 'cpo: erro_y: ', erro_y
					
					#y_pub = (y_kinect + 3*y_velodyne)/4
					#atribuir os pesos correto, pq no lado A tem grandes relevos
					#x_pub = (x_kinect + 3*x_velodyne)/4
					y_pub = (y_kinect + y_velodyne)/2
					#analisar qual dos dois valores pegar se erro for muito grande
					x_pub = (x_kinect + x_velodyne)/2
					
					pub_main = rospy.Publisher('/object_control', Float32MultiArray, queue_size = 1)
					a = Float32MultiArray()
					a.data = (objeto, x_pub, y_pub, 2, 180, 0, 0, 0)
					x_desvio_ref = x_pub
					y_desvio_ref = y_pub
					flag_retorno = 0
					while (flag_retorno != 1):
						print "cpo: publicando para cpmo desvio"
						pub_main.publish(a)
					flag_volta = 0
					flag_desvio = 1
					flag_retorno = 0
					flag_velodyne = 0
					x_kinect = 0
					y_kinect = 0
					x_velodyne = 0
					y_velodyne = 0
					
			elif(flag_desvio==1 and flag_volta == 0): #main_control.py jah remove da fila quando isso eh satisfeito, talvez aqui tambem tenha que fazer uma publicacao
				print 'cpo: desviando x_desvio_ref: ', x_desvio_ref 
				if( np.absolute(x_desvio_ref - gps_x) < 0.05 and np.absolute(y_desvio_ref - gps_y) < 0.05):
					flag_desvio = 0 #pode ter um problema de sincronismo, se adicionar na pilha o proximo elemento antes dela ter removido o anterior
					flag_volta = 1
					flag_retorno = 0
					print 'cpo: desviando TERMINADO'
					
			elif(flag_desvio==0 and flag_volta == 1):
				msg_l = rospy.Publisher('/OBC_OBV', Float32MultiArray, queue_size = 1)
				if(temp_x_ref == 0):
					x_velodyne = 0
					y_velodyne = 0
				print 'cpo: x_velodyne: ', x_velodyne 
				if (np.absolute(temp_x_ref - gps_x) < 0.05):
					if(gps_y>0):
						y_pub = 1.85
						x_pub = gps_x-2.0
					else:
						y_pub = -1.95
						x_pub = gps_x+2.0
					#analisar qual dos dois valores pegar se erro for muito grande
					pub_main = rospy.Publisher('/object_control', Float32MultiArray, queue_size = 1)
					a = Float32MultiArray()
					a.data = (objeto, x_pub, y_pub, 2, 180, 0, 0, 0)
					flag_retorno = 0
					flag_retorno = 0
					flag_desvio = 1
					flag_volta = 1
					flag_velodyne = 0
					x_kinect = 0
					y_kinect = 0
					x_velodyne = 0
					y_velodyne = 0
					temp_x_ref = x_pub
					temp_y_ref = y_pub
				else:
					print 'cpo: temp_x_ref', temp_x_ref, temp_y_ref
					if(temp_x_ref == 0 and temp_y_ref == 0 and x_velodyne==0): #tem uma limitacao aqui
						if(gps_y>0):
							temp_x_ref = gps_x - 0.5 #mudei de 0.25 para 1 para implementar a volta automatica
							temp_y_ref = gps_y
						
						else:
							temp_x_ref = gps_x + 0.5
							temp_y_ref = gps_y
						x_pub = temp_x_ref
						y_pub = temp_y_ref
						pub_main = rospy.Publisher('/object_control', Float32MultiArray, queue_size = 1)
						a = Float32MultiArray() #se a variacao no eixo x for menor que 30 cm pode ter um problema na tangente hiperbolica
						a.data = (objeto, x_pub, y_pub, 2, 180, 0, 0, 0)
						print 'cpo: antes de publicar. flag_retorno:', flag_retorno
			elif(flag_desvio == 1 and flag_volta == 1):
					print 'cpo: voltando'
					if(np.absolute(temp_x_ref - gps_x) < 0.03):
						print 'cpo: terminou volta'
						flag_desvio = 0
						flag_volta = 0
						x_kinect = 0
						y_kinect = 0
						x_velodyne = 0
						y_velodyne = 0
						
		elif(objeto==1):
			pub_main = rospy.Publisher('/object_control', Float32MultiArray, queue_size = 1)
			a = Float32MultiArray()
			a.data = (objeto, x_kinect, y_kinect, 2, 180, 0, 0, 0)
			print 'cpo: corredor'
			flag_retorno = 0
			#while (flag_retorno != 1 and x_kinect != 0):
			#	print "cpo: mandando para o cpmo do corredor", 'x: ', x_kinect, 'y: ', y_kinect
				#pub_main.publish(a)
			flag_retorno = 0
			flag_volta = 0
			flag_desvio = 0
			x_kinect = 0
			y_kinect = 0
			x_velodyne = 0
			y_velodyne = 0
	else:
		flag_volta = 0
		flag_desvio = 0
		flag_retorno = 0
		flag_velodyne = 0
		x_kinect = 0
		y_kinect = 0
		x_velodyne = 0
		y_velodyne = 0
		
def talker(): #Inits nodes and subscribers
	rospy.init_node('Object_Control', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/object_velodyne', Float32MultiArray, Velodyne)
	rospy.Subscriber('/object_detection', Float32MultiArray, Kinect)
	rospy.Subscriber('/main_control_feedback', Float32MultiArray, MC_Feedback)
	rospy.Subscriber('/velodyne_feedback', Float32MultiArray, Velodyne_Feedback)
	rospy.spin()
	
if __name__ == '__main__':
	try:
	    talker()
	except rospy.ROSInterruptException:
	    pass
