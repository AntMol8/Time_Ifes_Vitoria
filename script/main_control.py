#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Int8
from sensor_msgs.msg import NavSatFix

gps_x, x_ref = 0, 0
gps_y, y_ref = 0, 0
#goals = [(0.0, -2.2, 0, 180, 0, 0), (-1.5, -2.2, 2, 180, 0, 0) , (-1.5, -2.2, 0, 180, 0, 1),(-5.70, -2.2, 2, 180, 0, 0), (-11.9, -2.2, 2, 180, 0, 0),(-13.2, -3.0, 2, 180, 0, 0),(-14.7, -3.0, 2, 180, 0, 0),(-15.5, -1.95, 2, 180, 0, 0),(-20.0, -1.95, 2, 180, 0, 0),(-21.3, -2.4, 2, 180, 0, 0),(-22.0, -2.4, 2, 180, 0, 0),(-23.3, -1.85, 2, 180, 0, 0),(-39.4, -1.85, 2, 180, 0, 0),(-42.0, -3.4, 2, 180, 0, 0),(-52.0, -3.4, 2, 180, 0, 0),(-57.0, 3.4, 2, 180, 0, 0),(-42.0, 3.4, 2, 180, 0, 0),(-38.0, 1.85, -2, 180, 0, 0), (-41.6, 1.85, -2, 35, 35, 0), (-42.55, 1.85, -2, 180, 35, 2), (-42.9, 1.85, -2, 180, 0, 2), (-51.0, 1.85, 2, 180, 0, 2), (-50.0, 1.85, 0, 180, 0, 1), (-46.20, 1.85, 2, 180, 0, 2), (-43.00, 1.85, 3, 0, 45, 0), (-41.5, 1.85, 2, 180, 0, 0), (-9.0, 1.85, 0, 180, 0, 1), (-6.77, 1.85, 2, 180, 0, 0), (-9.0, 1.85, 0, 180, 0, 1), (-6.31, 1.85, 2, 180, 0, 0), (-3.0, 3.0, 2, 180, 0, 0)] #----antes das duas tangentes, (x,y,speed,frontal, traseiro,???)
#goals = [(0.0, -2.2, 0, 180, 0, 0), (-1.5, -2.2, 2, 180, 0, 0) , (-1.5, -2.2, 0, 180, 0, 1),(-5.70, -2.2, 2, 180, 0, 0), (-11.9, -2.2, 2, 180, 0, 0),(-13.2, -3.0, 2, 180, 0, 0),(-14.7, -3.0, 2, 180, 0, 0),(-15.5, -1.95, 2, 180, 0, 0),(-20.0, -1.95, 2, 180, 0, 0),(-21.3, -2.4, 2, 180, 0, 0),(-22.0, -2.4, 2, 180, 0, 0),(-23.3, -1.85, 2, 180, 0, 0),(-39.4, -1.85, 2, 180, 0, 0),(-42.0, -3.4, 2, 180, 0, 0),(-52.0, -3.4, 2, 0, 0, 0),(-56.0, 0, 2, 0, 0, 0),(-52.0, 3.4, 2, 0, 0, 0),(-42.0, 3.4, 2, 180, 0, 0),(-38.0, 1.85, -2, 180, 0, 0), (-41.6, 1.85, -2, 35, 35, 0), (-42.55, 1.85, -2, 180, 35, 2), (-42.9, 1.85, -2, 180, 0, 2), (-51.0, 1.85, 2, 180, 0, 2), (-50.0, 1.85, 0, 180, 0, 1), (-46.20, 1.85, 2, 180, 0, 2), (-43.00, 1.85, 3, 0, 45, 0), (-41.5, 1.85, 2, 180, 0, 0), (-9.0, 1.85, 0, 180, 0, 1), (-6.77, 1.85, 2, 180, 0, 0), (-9.0, 1.85, 0, 180, 0, 1), (-6.31, 1.85, 2, 180, 0, 0), (-3.0, 3.0, 2, 180, 0, 0)] #corrigido o final / index = len(goals)-15
goals = [(0.0, -2.2, 0, 180, 0, 0),(-1.0, -1.85, 2, 180, 0, 0),(-39.4, -1.85, 2, 180, 0, 0),(-42.0, -3.4, 2, 180, 0, 0),(-52.0, -3.4, 2, 0, 0, 0),(-56.0, 0, 2, 0, 0, 0),(-52.0, 3.4, 2, 0, 0, 0),(-42.0, 3.4, 2, 180, 0, 0),(-38.0, 1.85, -2, 180, 0, 0), (-41.6, 1.85, -2, 35, 35, 0), (-42.55, 1.85, -2, 180, 35, 2), (-42.9, 1.85, -2, 180, 0, 2), (-51.0, 1.85, 2, 180, 0, 2), (-43.00, 1.85, 3, 0, 45, 0), (-41.5, 1.85, 2, 180, 0, 0), (-6.31, 1.85, 2, 180, 0, 0), (-3.0, 3.0, 2, 180, 0, 0)]
index = len(goals)-2
x_master, y_master = 0, 0
objeto_anterior = 111
processando = 1 #eh false, 1 = 0 mudei para testar a fila
mudou = 1 # 0 = nao mudou
flag_retorno = 0
speed = 2
front_angle = 0
back_angle = 0
memory = 0
flug_arm=0

def BRACO(data):
	global goals, index, processando, memory
	
        if (data.data == 1 and (memory != data.data)):
        	index -= 1
		goals.pop(index)
		processando = 0
		
	memory = data.data
	
def RETORNO(data):
        global flag_retorno

	flag_retorno = data.data[0]

def ORDENS(data):
        global goals, index, mudou
        global x_master, y_master
        global objeto_anterior, flug_arm
        global speed, front_angle, back_angle
        
	msg_l = rospy.Publisher('/canaldocpmo', Float32MultiArray, queue_size = 1)
        pub_l = Float32MultiArray()
        print '-------------------'
        a_l = (1,0)
        pub_l.data = a_l
        msg_l.publish(pub_l)
        if (len(goals) == 0 and (x_master != data.data[0] or y_master != data.data[1])): #pode ser necessario adicionar um or (gps_x==x_master) para modificar as referencias
                x_master = data.data[0]
                y_master = data.data[1]
                speed = data.data[2]
                front_angle = data.data[3]
                back_angle = data.data[4]
                flug_arm = data.data[5]
                goals.insert(index, (x_objeto, y_objeto, speed, front_angle, back_angle, flug_arm))
                index += 1
                processando = 0
                mudou = 1

def SUBORDENS(data): #data.data[0] vai ser um contador de objetos
        global objeto_anterior, index, processando, mudou
        global speed, front_angle, back_angle, flug_arm
        global gps_x, gps_y

        if (objeto_anterior != data.data[0] and (-6 > gps_x > -51) ):
                x_objeto = data.data[1]
                y_objeto = data.data[2]
                speed = data.data[3]
                front_angle = data.data[4]
                back_angle = data.data[5]
                flug_arm = data.data[6]
                goals.insert(index, (x_objeto, y_objeto, speed, front_angle, back_angle, flug_arm))
                index += 1
                processando = 0
                objeto_anterior = data.data[0]
                mudou = 1
                
                
def GPS(data):
        global gps_x, gps_y, processando, index
        global mudou, objeto_anterior, x_ref, y_ref
        global flag_retorno
        global speed, front_angle, back_angle
	
	gps_x = data.latitude
	gps_y = data.longitude
	flag_retorno = 0
	flag_parar = 0
	print goals
	print len(goals)
	#rospy.loginfo('aaaaaaaaaaa')
	if(gps_y<0 and gps_x>-51.5 and goals[index - 1][0]<-51.5): #aparentemente foi resolvido, adicionar o opcao na condicao abaixo que corrige todos os problemas
		index -= 1
		goals.pop(index)
		processando = 0
	
	'''if(index==7 and gps_x>-14.0):
		index -= 1
		goals.pop(index)
		processando = 0'''
	
	if (len(goals) != 0):
		print goals[index - 1][5]
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
                        msg_a = rospy.Publisher('/canaldoarauto', Float32MultiArray, queue_size = 1)
                        pub_a = Float32MultiArray()
                        finalizado = 1
                        a_a = (finalizado)
                        pub_a.data = a_a
                        msg_a.publish(pub_a)
                        flag_parar = 1
                        mudou = 1
                        
        if (mudou == 1):     #precisa de um flag para quando for subir a escada
                msg_l = rospy.Publisher('/canaldolacaio', Float32MultiArray, queue_size = 1)
                pub_l = Float32MultiArray()
                print '-------------------'

                if (len(goals) != 0):
                        x_ref = goals[index-1][0]
                        y_ref = goals[index-1][1]
                        speed = goals[index-1][2]
                        front_angle = goals[index-1][3]
                        back_angle = goals[index-1][4]
                
                a_l = (objeto_anterior, flag_parar, x_ref, y_ref, speed, front_angle, back_angle)
                pub_l.data = a_l

                while (flag_retorno != 1 and len(goals) != 0):
                	print "-"
	                msg_l.publish(pub_l)
                        
	        flag_retorno = 0
                mudou = 0
                processando = 1
        

def talker():
	rospy.init_node('ARAUTO_DA_MOVIMENTACAO', anonymous = True)
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/canaldomestre', Float32MultiArray, ORDENS)
	rospy.Subscriber('/canaldocpo', Float32MultiArray, SUBORDENS)
	rospy.Subscriber('/canalderetornodolacaio', Float32MultiArray, RETORNO)
	rospy.Subscriber('/cpmo', Int8, BRACO, queue_size = 1)
	rospy.spin()
	
if __name__ == '__main__':
	try:
	    talker()
	except rospy.ROSInterruptException:
	    pass
