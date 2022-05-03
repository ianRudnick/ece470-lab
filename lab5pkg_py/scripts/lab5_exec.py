#!/usr/bin/env python

import sys
import copy
import time
import rospy

import numpy as np
from lab5_header import *
from lab5_func import *
from blob_search import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of green and yellow blocks
xw_yw_G = []
xw_yw_Y = []

# Any other global variable you want to define
# Hints: where to put the blocks?
xw_yw_G_init = []
xw_yw_Y_init = []


# ========================= Student's code ends here ===========================

################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False



"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	
	print("grip loc world: ", xWgrip, yWgrip)
	x_grip = xWgrip + 0.150
	y_grip = yWgrip - 0.150
	z_grip = zWgrip - 0.010
	yaw = np.radians(yaw_WgripDegree)

	print("grip loc: ", x_grip, y_grip, z_grip, yaw)

	L1 = 0.152
	L2 = 0.120
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059

	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0

	# Step 1
	x_center = x_grip - (L9 * np.cos(yaw))
	y_center = y_grip - (L9 * np.sin(yaw))
	z_center = z_grip

	# Step 2
	l1 = np.sqrt((x_center**2) + (y_center**2))
	l2 = L6 + 0.027
	print("a: ", l2/l1)
	theta1 = np.arctan2(y_center, x_center) - np.arcsin(l2 / l1)

	# Step 3
	theta6 = theta1 + (np.pi/2 - yaw)

	# Step 4
	delta = np.array([[-L7], [-l2], [L8+L10], [1]])
	center_to_base = np.array([
		[np.cos(theta1), -np.sin(theta1), 0, x_center],
		[np.sin(theta1),  np.cos(theta1), 0, y_center],
		[0, 0, 1, z_center], 
		[0, 0, 0, 1]])

	p_3end = np.matmul(center_to_base, delta)
	x_3end = p_3end[0][0]
	y_3end = p_3end[1][0]
	z_3end = p_3end[2][0]
	print("x_3end, y_3end, z_3end: ", x_3end, y_3end, z_3end)

	# Step 5
	a = np.sqrt((z_3end-L1)**2 + (x_3end)**2 + (y_3end)**2)
	theta3 = np.pi - np.arccos((L3**2 + L5**2 - a**2) / (2*L3*L5))
	theta_a = np.arccos((a**2 + L3**2 - L5**2) / (2*a*L3))
	theta_b = np.arcsin((z_3end - L1) / (a))
	theta_c = theta3 - theta_a
	theta_d = np.pi/2 - theta_b

	theta2 = -(theta_a + theta_b)
	theta4 = -(theta_c + theta_d - np.pi/2)

	theta5 = -np.pi/2

	# ==============================================================#
	print(theta1, theta2, theta3, theta4, theta5, theta6)

	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

	global digital_in_0
	digital_in_0 = msg.DIGIN
	digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

	global SPIN_RATE
	global thetas
	global current_io_0
	global current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			#rospy.loginfo("Goal is reached!")
			at_goal = 1

		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	global thetas
	global SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			#rospy.loginfo("Goal is reached!")

		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error

################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

	"""
	start_xw_yw_zw: where to pick up a block in global coordinates
	target_xw_yw_zw: where to place the block in global coordinates

	hint: you will use lab_invk(), gripper(), move_arm() functions to
	pick and place a block

	"""
	# ========================= Student's code starts here =========================

	# global variable1
	# global variable2

	error = 0

	# calc inv kinematics for each position
	start_thetas = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], 0)
	height_thetas = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], 0.120, 0)
	target_thetas = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0)
	target_height_thetas = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], 0.120, 0)

	# move arm to start and grip
	move_arm(pub_cmd, loop_rate, height_thetas, vel, accel)
	move_arm(pub_cmd, loop_rate, start_thetas, vel, accel)
	gripper(pub_cmd, loop_rate, True)
	move_arm(pub_cmd, loop_rate, height_thetas, vel, accel)
	# test if block was picked up/missing
	time.sleep(1.0)
	if (digital_in_0 < 1.0):
		print("Block missing in expected position!")
		error = 1
		gripper(pub_cmd, loop_rate, suction_off)

	# move arm to target and let go
	#move_arm(pub_cmd, loop_rate, go_away, vel, accel)
	move_arm(pub_cmd, loop_rate, target_height_thetas, vel, accel)
	move_arm(pub_cmd, loop_rate, target_thetas, vel, accel)
	gripper(pub_cmd, loop_rate, False)
	move_arm(pub_cmd, loop_rate, target_height_thetas, vel, accel)
	move_arm(pub_cmd, loop_rate, go_away, vel, accel)

	

	# ========================= Student's code ends here ===========================

	return error


class ImageConverter:

	def __init__(self, SPIN_RATE):

		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
		self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
		self.loop_rate = rospy.Rate(SPIN_RATE)

		# Check if ROS is ready for operation
		while(rospy.is_shutdown()):
			print("ROS is shutdown!")


	def image_callback(self, data):

		global xw_yw_G # store found green blocks in this list
		global xw_yw_Y # store found yellow blocks in this list

		try:
		  # Convert ROS image to OpenCV image
			raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv_image = cv2.flip(raw_image, -1)
		cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

		# You will need to call blob_search() function to find centers of green blocks
		# and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y respectively.

		# If no blocks are found for a particular color, you can return an empty list,
		# to xw_yw_G or xw_yw_Y.

		# Remember, xw_yw_G & xw_yw_Y are in global coordinates, which means you will
		# do coordinate transformation in the blob_search() function, namely, from
		# the image frame to the global world frame.

		xw_yw_G = blob_search(cv_image, "green")
		xw_yw_Y = blob_search(cv_image, "pink")



"""
Program run from here
"""
def main():

	global go_away
	global xw_yw_Y
	global xw_yw_G

	global xw_yw_G_init
	global xw_yw_Y_init

	# Initialize ROS node
	rospy.init_node('lab5node')

	# Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)
	sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	# Initialize the rate to publish to ur3/command
	loop_rate = rospy.Rate(SPIN_RATE)

	vel = 4.0
	accel = 4.0
	move_arm(pub_command, loop_rate, go_away, vel, accel)

	ic = ImageConverter(SPIN_RATE)
	time.sleep(5)

	# ========================= Student's code starts here =========================

	"""
	Hints: use the found xw_yw_G, xw_yw_Y to move the blocks correspondingly. You will
	need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
	"""
	rot = np.array([[np.cos(theta), -np.sin(theta)],
					[np.sin(theta), np.cos(theta)]]) 

	green_1_target = [0.135, -0.100, .035]
	green_2_target = [0.195, -0.155, .035]
	pink_1_target = [0.260, -0.100, .037]
	pink_2_target = [0.315, -0.155, .036]

	xw_yw_G_init = xw_yw_G
	xw_yw_Y_init = xw_yw_Y
	green1 = [xw_yw_G_init[0][0], xw_yw_G_init[0][1], 0.035]
	# print(green1)
	move_block(pub_command, loop_rate, green1, green_1_target, vel, accel)

	#xw_yw_G_init = xw_yw_G
	#xw_yw_Y_init = xw_yw_Y
	green2 = [xw_yw_G_init[1][0], xw_yw_G_init[1][1], 0.035]
	move_block(pub_command, loop_rate, green2, green_2_target, vel, accel)

	#xw_yw_G_init = xw_yw_G
	#xw_yw_Y_init = xw_yw_Y
	pink1 = [xw_yw_Y_init[0][0], xw_yw_Y_init[0][1], 0.036]
	move_block(pub_command, loop_rate, pink1, pink_1_target, vel, accel)

	#xw_yw_G_init = xw_yw_G
	#xw_yw_Y_init = xw_yw_Y
	pink2 = [xw_yw_Y_init[1][0], xw_yw_Y_init[1][1], 0.036]
	move_block(pub_command, loop_rate, pink2, pink_2_target, vel, accel)


	# ========================= Student's code ends here ===========================

	move_arm(pub_command, loop_rate, go_away, vel, accel)
	rospy.loginfo("Task Completed!")
	print("Use Ctrl+C to exit program")
	rospy.spin()

if __name__ == '__main__':

	try:
		main()
	# When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass
