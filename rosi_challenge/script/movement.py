#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import RosiMovement
from PIL import Image as IMG
import cv2

mapa = np.zeros([6000, 1000, 3], dtype = np.uint8) #coordenadas da esteira ---------vvvvvv, depois coloco
x1=int(-51.825*100+6000); x2=int(-2.15*100+6000)
y1=int(1.245*100+500);y2=int(-1.425*100+500)
cv2.rectangle(mapa, (y2, x2), (y1, x1), (255, 255, 255), -1)
x_gps, y_gps, z_gps = 0.0, 0.0, 0.0
x_ref, y_ref = -10,-2.2
back_arms_angle = 0
front_arms_angle = 35
speed = 2
#angle_reference = 0
horizontal_reference_gps = 1
z_quaternion_reference_gps =0

def Comunicao(data):
	global x_ref 
	global y_ref
	global front_arms_angle
	global back_arms_angle
	global speed
	
	speed = data.data[2]
	front_arms_angle = data.data[3]
	back_arms_angle = data.data[4]
	if (x_ref != data.data[0] or y_ref != data.data[1]):
		y_ref = data.data[1]
		x_ref = data.data[0]
		
		
def Escada(arms_joint_position): 		
	global front_arms_angle
	global back_arms_angle
	
	arm_right_front = arms_joint_position.movement_array[0].joint_var
	arm_right_back = arms_joint_position.movement_array[1].joint_var
	arm_left_front = arms_joint_position.movement_array[2].joint_var
	arm_left_back = arms_joint_position.movement_array[3].joint_var
	
	feedback = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size = 1)
	if(arm_right_front<(-np.pi/2)):
		arm_right_front = 2*np.pi + arm_right_front
	if(arm_left_front>(np.pi/2)):
		arm_left_front = -2*np.pi + arm_left_front
	front_arms_angle = front_arms_angle*np.pi / 180
	back_arms_angle = back_arms_angle*np.pi / 180
	ang_ref1 = front_arms_angle; ang_ref3 = -ang_ref1 		#ANGULOS DE REFERENCIA, positivo eh para cima
	ang_ref2 = back_arms_angle; ang_ref4 = -ang_ref2 		#negativo eh para cima

	erro1 = (arm_right_front-ang_ref1) * 10
	erro3 = (ang_ref3-arm_left_front) * 10
	erro2 = (arm_right_back-ang_ref2) * 10
	erro4 = (ang_ref4-arm_left_back) * 10
	#print '1: ', erro1, '2: ', erro2, '3: ', erro3, '4: ', erro4
	speed = 0;
	motor1 = RosiMovement(); motor1.nodeID = 1; motor1.joint_var = erro1
	motor2 = RosiMovement(); motor2.nodeID = 2; motor2.joint_var = erro2
	motor3 = RosiMovement(); motor3.nodeID = 3; motor3.joint_var = erro3
	motor4 = RosiMovement(); motor4.nodeID = 4; motor4.joint_var = erro4
	motor = RosiMovementArray();
	motor.movement_array = (motor1, motor2, motor3, motor4)
	feedback.publish(motor)
	
	front_arms_angle *= 180 / np.pi
	back_arms_angle *= 180 / np.pi

def GPS(GPS_data):
	global x_gps
	global y_gps
	global z_gps
	global horizontal_reference_gps
	global z_quaternion_reference_gps
	global x_ref
	global y_ref
	
	x_gps = GPS_data.latitude
	y_gps = GPS_data.longitude
	z_gps = GPS_data.altitude
	#print 'x: ', x_gps, 'y: ', y_gps
 	delta_y = y_ref - y_gps
	delta_x = x_ref - x_gps
	
	if (delta_x == 0):
		angle_reference = np.pi / 2
	else:
		angle_reference = delta_x/np.absolute(delta_x) * np.arctan(np.absolute(delta_y/delta_x))
		if (angle_reference < 0):
			angle_reference += np.pi
	if (delta_y < 0):
		horizontal_reference_gps = -1 		# eh false
	else:
		horizontal_reference_gps = 1
	#print "angle_reference: ", angle_reference
	#print "dx: ", delta_x, 'dy: ', delta_y 
	z_quaternion_reference_gps = np.sin(angle_reference / 2)
		
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
	global x_gps
	global y_gps
	global z_gps
	global y_ref
	global x_ref
	global mapa	
	global speed
	global z_quaternion_reference_gps
	global esquerda_ref, horizontal_reference_gps
	
	z_quaternion_reference = z_quaternion_reference_gps
	horizontal_reference = horizontal_reference_gps
	x = x_gps
	y = y_gps
	z = z_gps
	feedback = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size = 1)
	resolucao = 0.01
	mapa_x = int(x/resolucao) + 6000 
	mapa_y = int(y/resolucao) + 500
	
	if (mapa_x < 6000 and mapa_x >= 0) and (mapa_y < 1000 and mapa_y >= 0):
		if (abs(x_gps + 45.8) <= 0.1) and y_gps<0 and y_gps>-2:
			cv2.rectangle(mapa, (int(1/resolucao) + 500-5, int(-45.8/resolucao) + 6000-5), (int(1/resolucao) + 500+5, int(-45.8/resolucao) + 6000+5), (0, 0, 255), -1)
		if (abs(x_gps + 5.975) <= 0.1) and y_gps<0:
			cv2.rectangle(mapa, (int(-1.275/resolucao) + 500 - 5, int(-5.975/resolucao) + 6000 - 5), (int(-1.275/resolucao) + 500 +5, int(-5.975/resolucao) + 6000+5), (0, 0, 255), -1)
		mapa[mapa_x,mapa_y] = (255, 255, 255)
		cv2.imwrite('mapa.png', mapa)

	b = Imu_data.orientation.x
	c = Imu_data.orientation.y
	d = Imu_data.orientation.z
	a = Imu_data.orientation.w
	erro = 0.0  				
	if (a/np.absolute(a) * d < 0):
		esquerda_d = -1 		# eh false
	else:
		esquerda_d = 1
		
	a_dref = np.arcsin(z_quaternion_reference) * 2
	a_d = np.arcsin(np.absolute(d)) * 2
	#print 'a_dref: ', a_dref
	if(speed<0):
		horizontal_reference = horizontal_reference*speed/np.absolute(speed)		
		a_dref = np.pi - a_dref
		
	a_dir = horizontal_reference*a_dref - esquerda_d*a_d
	
	if (a_dir < 0):
		a_dir += 2*np.pi
		
	a_esq = 2*np.pi - a_dir
	
	constant=5
	
	if (a_dir < a_esq):
		erro = a_dir * constant
	else:
		erro = a_esq * -constant
		
	if(speed==0):
		erro=0
		erro12=0
		erro34=0
	elif(speed>0):
		if (erro > np.absolute(speed) ):
			erro12 = np.absolute(speed)
			erro34 = 3 * np.absolute(speed)
		elif (erro<-np.absolute(speed)):
			erro12 = -3 * np.absolute(speed)
			erro34 = -np.absolute(speed)
		else:
			erro12 = erro; erro34 = erro
	else:
		if (erro > np.absolute(speed) ):
			erro34 = np.absolute(speed)
			erro12 = 3 * np.absolute(speed)
		elif (erro<-np.absolute(speed)):
			erro34 = -3 * np.absolute(speed)
			erro12 = -np.absolute(speed)
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
	
	print "xref: ", x_ref, "yref: ", y_ref
	print 'x: ', x, 'y: ', y
	print "erro = d_ref - d: ", erro
	print "erro12: ", erro12
	print "erro34: ", erro34
	print "a_dir: ", a_dir
	print "a_esq: ", a_esq
	print "a_dref: ", a_dref
	print "horizontal_ref: ", horizontal_reference
	print "a_d = z: ", a_d
	print "esquerda_d: ", esquerda_d
	print '-------------------------'
	
def talker():
	rospy.init_node('Movement_Control')	
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

