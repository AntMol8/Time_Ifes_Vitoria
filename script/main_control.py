#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Int8
from sensor_msgs.msg import NavSatFix

gps_x, x_ref = 0, 0
gps_y, y_ref = 0, 0

#(x, y, speed, front angle, back angle, braco, recebe dos sensores)
#list of fixed goals for the robot that are: the corners of the conveyor belts, the stairs and rolls in the beginning of the conveyor belt 
goals = [(0.0, -2.2, 2, 180, 0, 0, 0, 0), (0.0, -1.2, 0, 180, 0, 1, 0),(-5.7, -2.2, 2, 180, 0, 0, 0),(-8.0, -1.95, 2, 180, 0, 0, 0),(-41.75, -2.01, 2, -45, -45, 0, 1),(-42.32, -2.01, 2, -45, 0, 0, 1),(-42.88, -2.01, 2, 0, 0, 2, 1),(-50.85, -2.01, -2, 0, 0, 2, 1),(-42.72, -2.01, -3, -45, 0, 0, 1),(-41.83, -2.01, -2, 0, -45, 0, 1), (-41.5, -2.01, -2, 0, 0, 0, 1), (-39.4, -2.01, 2, 180, 0, 0, 1),(-42.0, -3.4, 2, 180, 0, 0, 1),(-52.0, -3.4, 2, 0, 0, 0, 0),(-56.0, 0, 2, 0, 0, 0, 0),(-52.0, 3.4, 2, 0, 0, 0, 0),(-42.0, 3.4, 2, 180, 0, 0, 0),(-38.0, 1.85, -2, 180, 0, 0, 1), (-41.6, 1.85, -2, 35, 35, 0, 1), (-42.55, 1.85, -2, 180, 35, 2, 1), (-42.9, 1.85, -2, 180, 0, 2, 1), (-51.0, 1.85, 2, 180, 0, 2, 1), (-43.00, 1.85, 3, 0, 45, 0, 1), (-41.5, 1.85, 2, 180, 0, 0, 1), (-38, 1.85, 2, 180, 0, 0, 0),(-9.0, 1.85, 0, 180, 0, 1, 0), (-6.77, 1.85, 2, 180, 0, 0, 0), (-9.0, 1.85, 0, 180, 0, 1, 0), (-6.31, 1.85, 2, 180, 0, 0, 0), (-3.0, 3.0, 2, 180, 0, 0, 0)] #corrigir a subida de reh

index = len(goals)
x_master, y_master = 0, 0
previous_object = 111
global_prossessing = 1
global_changed = 1
return_flag = 0
speed = 2
front_angle = 0
back_angle = 0
memory = 0
arm_flag = 0

def Ur5_Feedback(data):
	#receives a flag when ur5 finishes the touching routine, so it can remove the top element of the list
	global goals, index, global_prossessing, memory
	
        if (data.data == 1 and (memory != data.data)):
        	index -= 1
		goals.pop(index)
		global_prossessing = 0
		
	memory = data.data
	
def CT_Feedback(data):
	#receives create_trajectory feedback so it can stop publishing information to it
        global return_flag

	return_flag = data.data[0]

def Kinect_Coordinate(data):
	#receives the coordinates from object_control and puts it in the list of goals
        global previous_object, index, global_prossessing, global_changed
        global speed, front_angle, back_angle, arm_flag, gps_x, gps_y
	
	msg_l = rospy.Publisher('/main_control_feedback', Float32MultiArray, queue_size = 1)
        pub_l = Float32MultiArray()
        pub_l.data = (1,0)
        msg_l.publish(pub_l)
	
        if ((goals[index - 1][0] != data.data[1] or goals[index - 1][1] != data.data[2]) and goals[index - 1][6] !=1):
		print 'cpmo: adicionado do cpo na fila'
                x_objeto = data.data[1]
                y_objeto = data.data[2]
                speed = data.data[3]
                front_angle = data.data[4]
                back_angle = data.data[5]
                arm_flag = data.data[6]
		
		if (gps_y > 0):
			goals.insert(index, (x_objeto - 2.5, 1.85, speed, front_angle, back_angle, arm_flag, data.data[7]))
			goals.insert(index + 1, (x_objeto - 1.0, y_objeto, speed, front_angle, back_angle, arm_flag, data.data[7]))
		else:
			goals.insert(index, (x_objeto + 1.5, 1.95, speed, front_angle, back_angle, arm_flag, data.data[7]))
			goals.insert(index + 1, (x_objeto + 0.5, y_objeto, speed, front_angle, back_angle, arm_flag, data.data[7]))
			
                goals.insert(index + 2, (x_objeto, y_objeto, speed, front_angle, back_angle, arm_flag, data.data[7]))
		
                index += 3
                global_prossessing = 0
                previous_object = data.data[0]
                global_changed = 1
                
                
def GPS(data): #Controls what elements should be removed from goals array by comparing the gps coordinates to the next goal of the robot
        global gps_x, gps_y, global_prossessing, index
        global global_changed, previous_object, x_ref, y_ref
        global return_flag
        global speed, front_angle, back_angle
	mudou = global_changed
	
	processing = global_prossessing
	gps_x = data.latitude
	gps_y = data.longitude
	return_flag = 0
	stop_flag = 0
	
	if (gps_y < 0 and gps_x > -51.5 and goals[index - 1][0] < -51.5):
		index -= 1
		goals.pop(index)
		processing = 0
	
	if (len(goals) != 0):
		flug = rospy.Publisher('/armBrain', Int8, queue_size = 1)
		flug.publish(goals[index - 1][5])
		
		if (processing == 1 and np.absolute(goals[index - 1][0] - gps_x) < 0.05) and (np.absolute(goals[index-1][1] - gps_y) < 0.05):
		        index -= 1
		        goals.pop(index)
		        processing = 0

	if (processing == 0):
                if (len(goals) != 0):
                        processing = 1
                        changed = 1
                else:
                        stop_flag = 1
                        changed = 1

        print 'cpmo:' , goals[index-1]
	print 'cpmo: ', index, len(goals)
          
        if (changed == 1):
                msg_l = rospy.Publisher('/main_control', Float32MultiArray, queue_size = 1)
                pub_l = Float32MultiArray()
                print 'cpmo: mandou para o lacaio', x_ref, y_ref

                if (len(goals) != 0):
                        x_ref = goals[index-1][0]
                        y_ref = goals[index-1][1]
                        speed = goals[index-1][2]
                        front_angle = goals[index-1][3]
                        back_angle = goals[index-1][4]
                
                a_l = (previous_object, stop_flag, x_ref, y_ref, speed, front_angle, back_angle)
                pub_l.data = a_l

                while (return_flag != 1 and len(goals) != 0):
                	print "cpmo: recebendo retorno do lacaio", x_ref, y_ref
	                msg_l.publish(pub_l)
                        
	        return_flag = 0
                changed = 0
                processing = 1
		
	global_prossessing = processing
	global_changed = changed
		
def Fire_Coordinate(data): #receives data from ur5Camera.py and sends info to array goals to start touch routine
	global goals, gps_y, gps_x, index, global_changed, global_prossessing
	
	x_roll = data.data[0]
	goals.insert(index, (gps_x + 1, gps_y + 1, 0, 180, 0, 1, 1))
	index += 1
        global_prossessing = 0
        previous_object = data.data[0]
        global_changed = 1

def talker(): #Inits nodes and declares subscribers
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
