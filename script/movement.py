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

gps_map = np.zeros([6000, 1200, 3], dtype = np.uint8)

#x1, x2, y1 and y2 represent the corners of the conveyor belt
x1, x2 = int(-51.825*100 + 6000), int(-2.15*100 + 6000)
y1, y2 = int(1.245*100 + 600), int(-1.425*100 + 600)

cv2.rectangle(gps_map, (y2, x2), (y1, x1), (255, 255, 255), -1)
x_gps, y_gps, z_gps = 0.0, 0.0, 0.0
x_ref, y_ref = 0, 0

back_arms_angle = 0
front_arms_angle = 0
speed = 2
horizontal_reference_gps = 1
z_quaternion_reference_gps = 0
resolution = 0

def Detect_Roll(data): # Receives data from detect_roll script and prints rolls on map
	global gps_map, resolution
	
	# gets rolls opening and closing edges
	x_roll_1 = int(data.data[0]/resolution) + 6000
	y_roll_1 = int(data.data[1]/resolution) + 600
	x_roll_2 = int(data.data[2]/resolution) + 6000
	y_roll_2 = int(data.data[3]/resolution) + 600

	# before painting the map, checks if the zone to be painted is not red so that it does not overlay the marked fires
	if ((gps_map[x_roll_1, y_roll_1] == (0, 0, 255)).all() or (gps_map[x_roll_2, y_roll_1] == (0, 0, 255)).all() or (gps_map[x_roll_1, y_roll_2] == (0, 0, 255)).all() or (gps_map[x_roll_2, y_roll_2] == (0, 0, 255)).all()):
		return
	# prints roll 	
	cv2.rectangle(gps_map, (int(data.data[1]/resolution) + 600 - 5, int(data.data[0]/resolution) + 6000 - 5), (int(data.data[3]/resolution) + 600 + 5, int(data.data[2]/resolution) + 6000 + 5), (0, 255, 0), -1)
	
def Detect_Fire(data): # receives data from ur5Camera with the fire's coordinates and prints fires on map
	global gps_map, resolution
	
	# fixes fire coordinates, compensating the difference between the ur5Camera and the gps
	if (y_gps > 1):
		a = data.data[0] + 0.136
	elif (y_gps < -1):
		a = data.data[0] - 0.136
	else:
		a = data.data[0]
	
	x_roll = a
	y_roll = data.data[1]
	
	# prints fire
	cv2.rectangle(gps_map, (int(y_roll / resolution) + 600 - 5, int(x_roll / resolution) + 6000 - 5), (int(y_roll / resolution) + 600 + 5, int(x_roll / resolution) + 6000 + 5), (0, 0, 255), -1)
	
def Coordinates_Control(data):
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
		
		
def Arms_Control(arms_joint_position): 		
	global front_arms_angle
	global back_arms_angle
	
	right_front_arm = arms_joint_position.movement_array[0].joint_var
	right_back_arm = arms_joint_position.movement_array[1].joint_var
	left_front_arm = arms_joint_position.movement_array[2].joint_var
	left_back_arm = arms_joint_position.movement_array[3].joint_var
	
	feedback = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size = 1)

	if (right_front_arm < (-np.pi / 2)):
		right_front_arm += 2 * np.pi
	if (left_front_arm > (np.pi / 2)):
		left_front_arm += -2 * np.pi
		
	front_arms_angle *= np.pi / 180
	back_arms_angle *= np.pi / 180
	ang_ref1, ang_ref3 = front_arms_angle, -front_arms_angle 		
	ang_ref2, ang_ref4 = back_arms_angle, -back_arms_angle		

	#calculates the difference between the ideal position and the actual position of the arms and amplifies it
	error1 = (right_front_arm-ang_ref1) * 10
	error3 = (ang_ref3-left_front_arm) * 10
	error2 = (right_back_arm-ang_ref2) * 10
	error4 = (ang_ref4-left_back_arm) * 10

	speed = 0
	front_right_engine = RosiMovement(); front_right_engine.nodeID = 1; front_right_engine.joint_var = error1
	back_right_engine = RosiMovement(); back_right_engine.nodeID = 2; back_right_engine.joint_var = error2
	front_left_engine = RosiMovement(); front_left_engine.nodeID = 3; front_left_engine.joint_var = error3
	back_left_engine = RosiMovement(); back_left_engine.nodeID = 4; back_left_engine.joint_var = error4
	engine = RosiMovementArray()
	engine.movement_array = (front_right_engine, back_right_engine, front_left_engine, back_left_engine)
	feedback.publish(engine)
	
	front_arms_angle *= 180 / np.pi
	back_arms_angle *= 180 / np.pi

def GPS(GPS_data): # receives data from GPS
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
 	delta_y = y_ref - y_gps
	delta_x = x_ref - x_gps
	
	if (delta_x == 0):
		angle_reference = np.pi / 2
	else:
		angle_reference = delta_x/np.absolute(delta_x) * np.arctan(np.absolute(delta_y/delta_x))
		
		if (angle_reference < 0):
			angle_reference += np.pi

	if (delta_y < 0):
		horizontal_reference_gps = -1 	
	else:
		horizontal_reference_gps = 1

	z_quaternion_reference_gps = np.sin(angle_reference / 2)
			
def Orientation_Control(Imu_data):
	global x_gps
	global y_gps
	global z_gps
	global y_ref
	global x_ref
	global gps_map	
	global speed
	global z_quaternion_reference_gps, resolution
	global horizontal_reference_gps
	#calculates orientation and adjust with the reference angle
	z_quaternion_reference = z_quaternion_reference_gps
	horizontal_reference = horizontal_reference_gps

	x = x_gps
	y = y_gps
	z = z_gps

	feedback = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size = 1)
	resolution = 0.01
	gps_map_x = int(x/resolution) + 6000 
	gps_map_y = int(y/resolution) + 600
	
	if (gps_map_x < 6000 and gps_map_x >= 0) and (gps_map_y < 1200 and gps_map_y >= 0):
		gps_map[gps_map_x,gps_map_y] = (255, 0, 0)
		cv2.imwrite('gps_map.png', gps_map)

	b = Imu_data.orientation.x
	c = Imu_data.orientation.y
	d = Imu_data.orientation.z
	a = Imu_data.orientation.w
	error = 0.0 

	if (a/np.absolute(a) * d < 0):
		left_angle = -1 
	else:
		left_angle = 1
		
	angle_ref = np.arcsin(z_quaternion_reference) * 2
	angle = np.arcsin(np.absolute(d)) * 2

	if (speed < 0):
		horizontal_reference *= speed / np.absolute(speed)		
		angle_ref = np.pi - angle_ref
		
	right_orientation_angle = horizontal_reference*angle_ref - left_angle*angle
	
	if (right_orientation_angle < 0):
		right_orientation_angle += 2*np.pi
		
	left_orientation_angle = 2*np.pi - right_orientation_angle
	
	constant = 5
	
	if (right_orientation_angle < left_orientation_angle):
		error = right_orientation_angle * constant
	else:
		error = -left_orientation_angle * constant
		
	if (speed == 0):
		error = 0
		error12 = 0
		error34 = 0
	elif (speed > 0):
		if (error > np.absolute(speed)):
			error12 = np.absolute(speed)
			error34 = 3 * np.absolute(speed)
		elif (error<-np.absolute(speed)):
			error12 = -3 * np.absolute(speed)
			error34 = -np.absolute(speed)
		else:
			error12 = error; error34 = error
	else:
		if (error > np.absolute(speed)):
			error34 = np.absolute(speed)
			error12 = 3 * np.absolute(speed)
		elif (error < -np.absolute(speed)):
			error34 = -3 * np.absolute(speed)
			error12 = -np.absolute(speed)
		else:
			error12 = error; error34 = error


	speed1, speed3, speed2, speed4 = speed, speed, speed, speed
	front_right_engine = RosiMovement(); front_right_engine.nodeID = 1; front_right_engine.joint_var = speed1 + error12
	back_right_engine = RosiMovement(); back_right_engine.nodeID = 2; back_right_engine.joint_var = speed2 + error12
	front_left_engine = RosiMovement(); front_left_engine.nodeID = 3; front_left_engine.joint_var = speed3 - error34
	back_left_engine = RosiMovement(); back_left_engine.nodeID = 4; back_left_engine.joint_var = speed4 - error34
	
	engine = RosiMovementArray()
	engine.movement_array = (front_right_engine, back_right_engine, front_left_engine, back_left_engine)
	feedback.publish(engine)
	
	print "xref: ", x_ref, "yref: ", y_ref
	print 'x: ', x, 'y: ', y
	print "error = d_ref - d: ", error
	print "error12: ", error12
	print "error34: ", error34
	print "right_orientation_angle: ", right_orientation_angle
	print "left_orientation_angle: ", left_orientation_angle
	print "angle_ref: ", angle_ref
	print "horizontal_ref: ", horizontal_reference
	print "angle = z: ", angle
	print "left_angle: ", left_angle
	print '-------------------------'
	
def talker(): # inits node and declares subscribers
	rospy.init_node('Movement_Control', anonymous = True)	
	rospy.Subscriber('/sensor/gps', NavSatFix, GPS)
	rospy.Subscriber('/trajectory_parameters', Float32MultiArray, Coordinates_Control)
	rospy.Subscriber('/detect_roll', Float32MultiArray, Detect_Roll)
	rospy.Subscriber('/detect_fire', Float32MultiArray, Detect_Fire)
	rospy.Subscriber('/rosi/arms_joints_position', RosiMovementArray, Arms_Control)
	rospy.Subscriber('/sensor/imu', Imu, Orientation_Control)
	rospy.spin()
	
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
