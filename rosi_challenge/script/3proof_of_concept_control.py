#!/usr/bin/env python
from rosi_defy.msg import ManipulatorJoints
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import cv2
import math
import numpy as np
import rospy

gatheringPosition = 0
relativePos = 0
touched = 0
armState = [0,0,0,0,0,0]
ur5BaseOrientation,ur5Shoulder,ur5Elbow,ur5Wrist,ur5PointerOrientation,ur5PointerRotation = 0,0,0,0,0,0
ang = 0
pointer = 0
orientation = 0
baseOrientation = 0
terrainCorrection = 0

def listener():
	rospy.init_node('arm_joints_states')
	rospy.Subscriber('/ur5/forceTorqueSensorOutput', TwistStamped, stopMovement)
	rospy.Subscriber('/sensor/imu', Imu, orientation)
	rospy.Subscriber('/sensor/gps', NavSatFix, relationToTransporter)
	rospy.spin()
	
def relationToTransporter(data):
	global relativePos	
	yPos = float(data.longitude)
	if yPos > 0:
		relativePos = -1			# transporter is on robot's left side
	else:
		relativePos = 1				# transporter is on robot's right side

		
def orientation(data):
	global pointer
	global orientation
	global baseOrientation
	global relativePos
	global terrainCorrection
	
	X = float(data.orientation.x)
	Y = float(data.orientation.y)
	Z = float(data.orientation.z)
	W = float(data.orientation.w)
	
	if (np.absolute(Z) < np.sqrt((1-X**2-Y**2)/2)):					# esta apontando para X negativo
		if relativePos == 1:
			pointer = 0
		else:	
			pointer = math.radians(180)
		orientation=-1
		baseOrientation=math.radians(90)*relativePos
	else:										# esta apontando para X positivo
		if relativePos == 1:
			pointer = math.radians(180)
		else:	
			pointer = 0
		orientation=1
		baseOrientation=math.radians(-90)*relativePos
		
	quat_to_radians1= (np.arctan(2*(X*Y + Z*W)))/(1-2*(Y**2+Z**2))
	quat_to_radians2 = np.arcsin(2*(X*Z - W*Y))
	
	if(quat_to_radians2 > math.pi/4):
		quat_to_radians2 = -math.pi/4
		
	if(quat_to_radians2 < -math.pi/4):
		quat_to_radians2 = math.pi/4 
	terrainCorrection = quat_to_radians2*orientation
	pointer=pointer*orientation
	
def stopMovement(data):
	global touched	
	global armstate
	global orientation
	global ang
	global pointer
	global baseOrientation
	global terrainCorrection
	global gatheringPosition
	ray = ManipulatorJoints()
	
	
	pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size = 1)

	x = round(float(data.twist.linear.x),2)
	y = round(float(data.twist.linear.y),2) 
	z = round(float(data.twist.linear.z),2)
	
	if touched < 20:
		ray.joint_variable = [baseOrientation,0,0,0,math.radians(-90),0]
		touched += 1
	
	if (x > 0.1 or y > 0.1 or z > 0.1) and touched == 20:
		touched = 21
			
	if touched == 20:
		ang += math.radians(.35)
		ray.joint_variable = [baseOrientation,ang,0,-ang,math.radians(-90),0]

	elif ang > 0: 
		ang -= math.radians(1)
		ray.joint_variable = [baseOrientation,ang,0,-ang,math.radians(-90),0]
		gatheringPosition = 1
				
	elif gatheringPosition == 1:
		ray.joint_variable = (pointer,terrainCorrection,0,0,0,-terrainCorrection)
			
	#print ray.joint_variable
	pub.publish(ray)	
		
if __name__ == "__main__":
	try:	
		listener()
	except rospy.ROSInterruptException:
		print "Exception occured. Shutting down."	
