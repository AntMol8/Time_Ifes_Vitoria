#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import RosiMovement
from PIL import Image as IMG

mapa = np.zeros([1000, 1000], dtype = np.uint8)
mapa[..., 500] = 255
x, y, z = 0.0, 0.0, 0.0
x_ref, y_ref = -30.0, -3.0          #1.87
orientation = 1
flag = 0; ref_ant = 0; flagy = 0
angulo = -0
speed = 2
angulo_ref = 0
esquerda_ref = 1

def Comunicao(data):
	global x_ref 		#na passagem do caminho estreito a correia pode ser um problema, se a roda prender nela
	global y_ref
	global flag
	global flagy
	global angulo
	global speed
	
	speed = data.data[2]
	angulo = data.data[3]
	
	if (x_ref != data.data[0] or y_ref != data.data[1]):
		y_ref = data.data[1]
		x_ref = data.data[0]
		flag = 0; ref_ant = 0; flagy = 0
		
def Escada(Angulo): 		#no x = -40 parece bom para comecar o processo de subir
	global angulo
	
	eixo1 = Angulo.movement_array[0].joint_var
	eixo2 = Angulo.movement_array[1].joint_var
	eixo3 = Angulo.movement_array[2].joint_var
	eixo4 = Angulo.movement_array[3].joint_var
	
	feedback = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size = 1)
	
	angulo = angulo*np.pi / 180
	ang_ref1 = angulo; ang_ref3 = -ang_ref1 		#ANGULOS DE REFERENCIA, positivo eh para cima
	ang_ref2 = angulo; ang_ref4 = -ang_ref2 		#negativo eh para cima

	erro1 = (eixo1-ang_ref1) * 15
	erro3 = (ang_ref3-eixo3) * 15
	erro2 = (eixo2-ang_ref2) * 15
	erro4 = (ang_ref4-eixo4) * 15
	
	speed1 = 0; speed2 = 0; speed3 = 0; speed4 = 0
	motor1 = RosiMovement(); motor1.nodeID = 1; motor1.joint_var = speed1 + erro1
	motor2 = RosiMovement(); motor2.nodeID = 2; motor2.joint_var = speed2 + erro2
	motor3 = RosiMovement(); motor3.nodeID = 3; motor3.joint_var = speed3 + erro3
	motor4 = RosiMovement(); motor4.nodeID = 4; motor4.joint_var = speed4 + erro4
	motor = RosiMovementArray();
	motor.movement_array = (motor1, motor2, motor3, motor4)
	feedback.publish(motor)
	
	angulo *= 180 / np.pi

def GPS(GPS_data):
	global x
	global y
	global z
	global esquerda_ref
	global d_ref
	global x_ref
	global y_ref
	
	x = GPS_data.latitude
	y = GPS_data.longitude
	z = GPS_data.altitude
	
	#y_ref=input("y_ref: ")
	#x_ref=input("x_ref: ")
	
	delta_y = y_ref - y
	delta_x = x_ref - x
	
	if (delta_x == 0):
		angulo_ref = np.pi / 2
	else:
		angulo_ref = delta_x/np.absolute(delta_x) * np.arctan(np.absolute(delta_y/delta_x))
		if (angulo_ref < 0):
			angulo_ref += np.pi
	if (delta_y < 0):
		esquerda_ref = -1 		# eh false
	else:
		esquerda_ref = 1
		
	d_ref = np.sin(angulo_ref / 2)
		
def Parar():
	feedback = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size = 1)
	motor1 = RosiMovement(); motor1.nodeID = 1; motor1.joint_var = 0.0
	motor2 = RosiMovement(); motor2.nodeID = 2; motor2.joint_var = 0.0
	motor3 = RosiMovement(); motor3.nodeID = 3; motor3.joint_var = 0.0
	motor4 = RosiMovement(); motor4.nodeID = 4; motor4.joint_var = 0.0
	motor = RosiMovementArray()
	motor.movement_array = (motor1, motor2, motor3, motor4)
	feedback.publish(motor)
	rospy.sleep(2)
	
def Imuu(Imu_data):
	global x
	global y
	global z 
	global y_ref
	global x_ref
	global mapa	
	global orientation
	global flag
	global ref_ant
	global flagy
	global speed
	global d_ref
	global esquerda_ref
	
	feedback = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size = 1)
	resolucao = 0.01
	mapa_x = int(x/resolucao) + 500 
	mapa_y = int(y/resolucao) + 500
	
	if (mapa_x < 1000 and mapa_x >= 0) and (mapa_y < 1000 and mapa_y >= 0):
		mapa[mapa_x,mapa_y] = 255
		imagem = IMG.fromarray(mapa, 'L')
		imagem.save('caminho.png', "PNG")

	b = Imu_data.orientation.x
	c = Imu_data.orientation.y
	d = Imu_data.orientation.z
	a = Imu_data.orientation.w
	erro = 0.0  				#para aumentar a precisao basta aumentar o valor da constante 20
						#parece ser mais adequado usar o angulo no erro
	if (a/np.absolute(a) * d < 0):
		esquerda_d = -1 		# eh false
	else:
		esquerda_d = 1
		
	a_dref = np.arcsin(d_ref) * 2
	a_d = np.arcsin(np.absolute(d)) * 2
	a_dir = esquerda_ref*a_dref - esquerda_d*a_d
	
	if (a_dir < 0):
		a_dir += 2*np.pi
		
	a_esq = 2*np.pi - a_dir
	
	if (a_dir < a_esq):
		erro = a_dir * 15
	else:
		erro = a_esq * -15
	
	if (erro > speed):
		erro12 = speed
		erro34 = 3 * speed
	elif (erro<-speed):
		erro12 = -3 * speed
		erro34 = -speed
	else:
		erro12 = erro; erro34 = erro
	
	speed1, speed3, speed2, speed4 = speed, speed, speed, speed
	motor1 = RosiMovement(); motor1.nodeID = 1; motor1.joint_var = speed1 + erro12
	motor2 = RosiMovement(); motor2.nodeID = 2; motor2.joint_var = speed2 + erro12
	motor3 = RosiMovement(); motor3.nodeID = 3; motor3.joint_var = speed3 - erro34
	motor4 = RosiMovement(); motor4.nodeID = 4; motor4.joint_var = speed4 - erro34
	
	motor = RosiMovementArray()
	motor.movement_array = (motor1, motor2, motor3, motor4)
	feedback.publish(motor)
	
'''	print "xref: ", x_ref, "yref: ", y_ref
	print "erro = d_ref - d: ", erro
	print "erro12: ", erro12
	print "erro34: ", erro34
	print "a_dir: ", a_dir
	print "a_esq: ", a_esq
	print "a_dref: ", a_dref
	print "esquerda_ref: ", esquerda_ref
	print "a_d = z: ", a_d
	print "esquerda_d: ", esquerda_d
	print '-------------------------' '''
	
def talker():
	rospy.init_node('Subidorderampa')	
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/canaldecomunicao', Float32MultiArray, Comunicao)
	rospy.Subscriber('/rosi/arms_joints_position', RosiMovementArray, Escada)
	rospy.Subscriber('/sensor/imu', Imu, Imuu)
	rospy.spin()
	
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
