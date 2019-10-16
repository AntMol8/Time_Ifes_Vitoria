#!/usr/bin/env python
from rosi_defy.msg import ManipulatorJoints	
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int8
from sensor_msgs.msg import NavSatFix
import cv2
import math
import numpy as np
import rospy

ran = 0										# flags if touch routine has been run.
gatheringPosition = 0						# indicates if robot is on its gathering position.
relativePos = 0								# robot position relative to the conveyor belt, can be 1 or -1.
touched = 0									# indicates to the robot when it touches something with the ur5 manipulator sensor.
armState = [0,0,0,0,0,0]
ur5BaseOrientation, ur5Shoulder, ur5Elbow, ur5Wrist, ur5PointerOrientation, ur5PointerRotation = 0, 0, 0, 0, 0, 0
pointer = 0									# ur5 pointer orientation.
orientation = 0								# robot orientation.
terrainCorrection = 0						# ur5 manipulator compesation for terrain irregularities.
flag = 0									# defines robot current state to other scripts. 0 = compensating for terrain, 1 = touch routine,
											# 2 = ur5 pointer angled upwards.
x, y, z = 0, 0, 0
memory = 0
mapa = np.zeros([6000, 1000], dtype = np.uint8)
yawCorrection = 0
def listener():	
	rospy.init_node('arm_joints_states')
	rospy.Subscriber('/armBrain', Int8, interpreter)
	rospy.Subscriber('/ur5/forceTorqueSensorOutput', TwistStamped, getForce)
	rospy.Subscriber('/sensor/imu', Imu, orientation)
	rospy.Subscriber('/sensor/gps', NavSatFix, relationToTransporter)

	rospy.spin()
		
def relationToTransporter(data):
	global relativePos						# detects robot's position relative to the conveyor belt
	yPos = float(data.longitude)
		 
	if yPos > 0:
		relativePos = -1					# transporter is on robot's left side
	else:
		relativePos = 1						# transporter is on robot's right side

		
def orientation(data):						# reads robot's orientation published by the IMU
	global yawCorrection					# UPDATE
	global pointer
	global orientation
	global relativePos
	global terrainCorrection
	
	X = float(data.orientation.x)
	Y = float(data.orientation.y)
	Z = float(data.orientation.z)
	W = float(data.orientation.w)
	
	if (np.absolute(Z) < np.sqrt((1-X**2-Y**2)/2)):		# points towards negative X
		if relativePos == 1:
			pointer = 0
		else:	
			pointer = math.radians(180)
		orientation=-1
	else:												# points towards positive X
		if relativePos == 1:
			pointer = math.radians(180)
		else:	
			pointer = 0
		orientation=1
		
	quat_to_radians1= (np.arctan(2*(X*Y + Z*W)))/(1-2*(Y**2+Z**2))		# quaternion output to radian input transformation
	quat_to_radians2 = np.arcsin(2*(X*Z - W*Y))
	
	if(quat_to_radians2 > math.pi/4):
		quat_to_radians2 = -math.pi/4
		
	if(quat_to_radians2 < -math.pi/4):
		quat_to_radians2 = math.pi/4			 
	terrainCorrection = quat_to_radians2*orientation					# terrain correction definition relative to robot's orientation
	pointer=pointer*orientation											# pointer state definition relative to robot's orientation
	
	firstTerm = 2*((X*Y)+(Z*W))										# UPDATE
	secondTerm = (1-(2*((Z*Z)+(W*W))))								# UPDATE
	yawCorrection = -np.arctan(firstTerm/secondTerm)*orientation	# UPDATE
	
def getForce(data):
	global x, y, z
	x = round(float(data.twist.linear.x),2)
	y = round(float(data.twist.linear.y),2) 
	z = round(float(data.twist.linear.z),2)
	if abs(z) > 1:
		print 'Warning: Force exceeded 1.0 N. -- ', z

def interpreter(changeReadValue):
	global terrainCorrection
	global x, y, z, memory
	global yawCorrection	
	if(changeReadValue.data == 1 and memory!=changeReadValue.data):
		ray = ManipulatorJoints()
		pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size = 1)
		ray.joint_variable = [yawCorrection, terrainCorrection, 0, 0, math.radians(-90), 0]
		pub.publish(ray)
		ang = 0
		rospy.sleep(1.5)		
		while z < 0.2:
			ray.joint_variable = [yawCorrection + math.radians(90), terrainCorrection + ang, 0, -ang, math.radians(-90), 0]
			ang += 0.001
			pub.publish(ray)
			rospy.sleep(0.2)
			print '1:' , ang
			
		while ang > 0:
			print '2:' ,ang
			ang -= 0.001
			ray.joint_variable = [yawCorrection + math.radians(90), terrainCorrection + ang, 0, -ang, math.radians(-90), 0]
			pub.publish(ray)
			rospy.sleep(0.2)
		
		ray.joint_variable = [yawCorrection, terrainCorrection, 0, 0, 0, 0]
		pub.publish(ray)

		for inn in range(10000):
			flug = rospy.Publisher('/cpmo', Int8, queue_size = 1)
			flug.publish(1)

		for inn in range(10000):
			flug = rospy.Publisher('/cpmo', Int8, queue_size = 1)
			flug.publish(0)
			
	elif(changeReadValue.data==0):
		ray = ManipulatorJoints()
		pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size = 1)
		ray.joint_variable = (yawCorrection, terrainCorrection, 0,0,0,0)
		pub.publish(ray)
		
	elif(changeReadValue.data==2):
		ray = ManipulatorJoints()
		pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size = 1)
		ray.joint_variable = (yawCorrection + math.radians(90), 0, 0, math.radians(-20), math.radians(-90), 0)
		pub.publish(ray)
	
	memory = changeReadValue.data
			
		
if __name__ == "__main__":
	try:	
		listener()
	except rospy.ROSInterruptException:
		print "Exception occured. Shutting down."	
