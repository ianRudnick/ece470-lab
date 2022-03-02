#!/usr/bin/env python
from turtle import end_fill
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	# @position_callback(msg)
	# L1 = .152 
	# L2 = .120 
	# L3 = .244
	# L4 = .093
	# L5 = .213
	# L6 = .083
	# L7 = .083
	# L8 = .082

	# q and omega for calculating each joint's S matrix
	q = np.array([[-150, 150, 10], [-150, 270, 162], [94, 270, 162], [307, 177, 162], [307, 260, 162], [390, 260, 162]])
	w = np.array([[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0]])

	S1 = np.zeros(shape=(4,4))
	S2 = np.zeros(shape=(4,4))
	S3 = np.zeros(shape=(4,4))
	S4 = np.zeros(shape=(4,4))
	S5 = np.zeros(shape=(4,4))
	S6 = np.zeros(shape=(4,4))
	S = np.array([S1, S2, S3, S4, S5, S6])

	for i in range(6):
		w_matrix = np.array([[0,-w[i][2],w[i][1]], [w[i][2],0,-w[i][0]], [-w[i][1],w[i][0],0]])
		v = np.cross(-w[i], q[i])
	
		S[i] = np.vstack([np.hstack([w_matrix, v.reshape((3,1))]), [0, 0, 0, 0]])

	# M matrix of end effector, precalculated
	M = np.array([[0, -1, 0, 390], [0, 0, -1, 401], [1, 0, 0, 215.5], [0, 0, 0, 1]])


	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	# L1 = 152 #mm
	# L2 = 120 
	# L3 = 244
	# L4 = 93
	# L5 = 213
	# L6 = 83
	# L7 = 83
	# L8 = 82

	M, S = Get_MS()

	T = expm(np.multiply(S[0], theta1))
	T = np.matmul(T, expm(np.multiply(S[1], theta2)))
	T = np.matmul(T, expm(np.multiply(S[2], theta3)))
	T = np.matmul(T, expm(np.multiply(S[3], theta4)))
	T = np.matmul(T, expm(np.multiply(S[4], theta5)))
	T = np.matmul(T, expm(np.multiply(S[5], theta6)))

	T = np.matmul(T, M)

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
