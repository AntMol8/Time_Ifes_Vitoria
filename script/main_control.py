#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Int8
from sensor_msgs.msg import NavSatFix

gps_x, x_ref = 0, 0
gps_y, y_ref = 0, 0
#(x,y,speed,frontal, traseiro,braco, recebe dos sensores)
goals = [(0.0, -2.2, 2, 180, 0, 0, 0, 0), (0.0, -1.2, 0, 180, 0, 1, 0),(-5.7, -2.2, 2, 180, 0, 0, 0),(-8.0, -1.95, 2, 180, 0, 0, 0),(-41.75, -2.01, 2, -45, -45, 0, 1),(-42.32, -2.01, 2, -45, 0, 0, 1),(-42.88, -2.01, 2, 0, 0, 2, 1),(-50.85, -2.01, -2, 0, 0, 2, 1),(-42.72, -2.01, -3, -45, 0, 0, 1),(-41.83, -2.01, -2, 0, -45, 0, 1), (-41.5, -2.01, -2, 0, 0, 0, 1), (-39.4, -2.01, 2, 180, 0, 0, 1),(-42.0, -3.4, 2, 180, 0, 0, 1),(-52.0, -3.4, 2, 0, 0, 0, 0),(-56.0, 0, 2, 0, 0, 0, 0),(-52.0, 3.4, 2, 0, 0, 0, 0),(-42.0, 3.4, 2, 180, 0, 0, 0),(-38.0, 1.85, -2, 180, 0, 0, 1), (-41.6, 1.85, -2, 35, 35, 0, 1), (-42.55, 1.85, -2, 180, 35, 2, 1), (-42.9, 1.85, -2, 180, 0, 2, 1), (-51.0, 1.85, 2, 180, 0, 2, 1), (-43.00, 1.85, 3, 0, 45, 0, 1), (-41.5, 1.85, 2, 180, 0, 0, 1), (-38, 1.85, 2, 180, 0, 0, 0),(-9.0, 1.85, 0, 180, 0, 1, 0), (-6.77, 1.85, 2, 180, 0, 0, 0)(-9.0, 1.85, 0, 180, 0, 1, 0), (-6.31, 1.85, 2, 180, 0, 0, 0), (-3.0, 3.0, 2, 180, 0, 0, 0)] #corrigir a subida de reh

#-6.77, 1.85, 2, 180, 0, 0, 1
index = len(goals)-3
x_master, y_master = 0, 0
objeto_anterior = 111
processando_global = 1 #eh false, 1 = 0 mudei para testar a fila
mudou_global = 1 # 0 = nao mudou
flag_retorno = 0
speed = 2
front_angle = 0
back_angle = 0
memory = 0
flug_arm=0

def Ur5_Feedback(data):
	global goals, index, processando_global, memory
	
        if (data.data == 1 and (memory != data.data)):
        	index -= 1
		goals.pop(index)
		processando_global = 0
		
	memory = data.data
	
def CT_Feedback(data):
        global flag_retorno

	flag_retorno = data.data[0]

def Kinect_Coordinate(data): #data.data[0] vai ser um contador de objetos
        global objeto_anterior, index, processando_global, mudou_global
        global speed, front_angle, back_angle, flug_arm, gps_x, gps_y
	msg_l = rospy.Publisher('/main_control_feedback', Float32MultiArray, queue_size = 1)
        pub_l = Float32MultiArray()
        a_l = (1,0)
        pub_l.data = a_l
        msg_l.publish(pub_l)
        if ( (goals[index - 1][0] != data.data[1] or goals[index - 1][1] != data.data[2]) and goals[index - 1][6] !=1): #analisar se isso eh generalizado
		print 'cpmo: adicionado do cpo na fila'
                x_objeto = data.data[1]
                y_objeto = data.data[2]
                speed = data.data[3]
                front_angle = data.data[4]
                back_angle = data.data[5]
                flug_arm = data.data[6]
		if(gps_y>0):
			goals.insert(index, (x_objeto-2.5, 1.85, speed, front_angle, back_angle, flug_arm, data.data[7]))
			goals.insert(index+1, (x_objeto-1.0, y_objeto, speed, front_angle, back_angle, flug_arm, data.data[7]))
		else:
			goals.insert(index, (x_objeto+1.5, 1.95, speed, front_angle, back_angle, flug_arm, data.data[7]))
			goals.insert(index+1, (x_objeto+0.5, y_objeto, speed, front_angle, back_angle, flug_arm, data.data[7]))
                goals.insert(index+2, (x_objeto, y_objeto, speed, front_angle, back_angle, flug_arm, data.data[7]))
		
                index += 3
                processando_global = 0
                objeto_anterior = data.data[0]
                mudou_global = 1
                
                
def GPS(data):
        global gps_x, gps_y, processando_global, index
        global mudou_global, objeto_anterior, x_ref, y_ref
        global flag_retorno
        global speed, front_angle, back_angle
	mudou = mudou_global
	processando = processando_global
	gps_x = data.latitude
	gps_y = data.longitude
	flag_retorno = 0
	flag_parar = 0
	if(gps_y<0 and gps_x>-51.5 and goals[index - 1][0]<-51.5): #aparentemente foi resolvido, adicionar o opcao na condicao abaixo que corrige todos os problemas
		index -= 1
		goals.pop(index)
		processando = 0
	
	'''if(index==7 and gps_x>-14.0):
		index -= 1
		goals.pop(index)
		processando = 0'''
	
	if (len(goals) != 0):
		flug = rospy.Publisher('/armBrain', Int8, queue_size = 1)
		flug.publish(goals[index - 1][5])
		
		if (processando == 1 and np.absolute(goals[index - 1][0] - gps_x) < 0.05) and (np.absolute(goals[index-1][1] - gps_y) < 0.05):
		        index -= 1
		        goals.pop(index)
		        processando = 0

	if (processando == 0):
                if (len(goals) != 0):
                        processando = 1
                        mudou = 1
                else:
                        #msg_a = rospy.Publisher('/canaldoarauto', Float32MultiArray, queue_size = 1)
                        #pub_a = Float32MultiArray()
                        #finalizado = 1
                        #a_a = (finalizado)
                        #pub_a.data = a_a
                        #msg_a.publish(pub_a)
                        flag_parar = 1
                        mudou = 1

        print 'cpmo:' , goals[index-1]
	print 'cpmo: ', index, len(goals)
          
        if (mudou == 1):     #precisa de um flag para quando for subir a escada
                msg_l = rospy.Publisher('/main_control', Float32MultiArray, queue_size = 1)
                pub_l = Float32MultiArray()
                print 'cpmo: mandou para o lacaio', x_ref, y_ref

                if (len(goals) != 0):
                        x_ref = goals[index-1][0]
                        y_ref = goals[index-1][1]
                        speed = goals[index-1][2]
                        front_angle = goals[index-1][3]
                        back_angle = goals[index-1][4]
                
                a_l = (objeto_anterior, flag_parar, x_ref, y_ref, speed, front_angle, back_angle)
                pub_l.data = a_l

                while (flag_retorno != 1 and len(goals) != 0):
                	print "cpmo: recebendo retorno do lacaio", x_ref, y_ref
	                msg_l.publish(pub_l)
                        
	        flag_retorno = 0
                mudou = 0
                processando = 1
	processando_global = processando
	mudou_global = mudou
		
def Fire_Coordinate(data):
	global goals, gps_y, gps_x, index, mudou_global, processando_global
	x_rolo = data.data[0]
	goals.insert(index, (gps_x + 1, gps_y + 1, 0, 180, 0, 1, 1))
	index += 1
        processando_global = 0
        objeto_anterior = data.data[0]
        mudou_global = 1

def talker():
	rospy.init_node('Main_Control', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/object_control', Float32MultiArray, Kinect_Coordinate)
	rospy.Subscriber('/detect_fire', Float32MultiArray, Fire_Coordinate)
	rospy.Subscriber('/create_trajectory_feedback', Float32MultiArray, CT_Feedback)
	rospy.Subscriber('/cpmo', Int8, Ur5_Feedback)
	rospy.spin()
	
if __name__ == '__main__':
	try:
	    talker()
	except rospy.ROSInterruptException:
	    pass
