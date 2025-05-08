#!/usr/bin/env python

'''

lab7pkg_pick_place/lab_func.py

@brief: functions for computing forward and inverse kinematics of UR3e robot arm
@author: Songjie Xiao
@date: Monday 2023/3/20

'''

import numpy as np
import math
from scipy.linalg import expm

PI = np.pi

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
Add any helper functions as you need.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	# =====================Define Matrix M===========================#
	M[0][0] = 0
	M[0][2] = 1 #f0 x axis in f_end
	M[0][3] = (120-93+104+92+70)/1000 #M_Px in meter
	M[1][3] = (-542)/1000 #M_Py in meter
	M[2][0] = -1 #f0 z axis in f_end
	M[2][2] = 0
	M[2][3] = 152/1000
    
	# =====================Define w===========================#
	w1= np.array([0,0,1])
	w2= np.array([1,0,0])    
	w3= np.array([1,0,0])  
	w4= np.array([1,0,0])
	w5= np.array([0,-1,0])
	w6= np.array([1,0,0])     
    
	# =====================Define q===========================#
	q1 = np.array([0, 0, 152])/1000
	q2 = np.array([120, 0, 152])/1000
	q3 = np.array([120, -244, 152])/1000   
	q4 = np.array([120-93, -244-213, 152])/1000
	q5 = np.array([120-93+104, -244-213, 152])/1000
	q6 = np.array([120-93+104, -542, 152])/1000   

	# =====================Define v===========================#
	v1 = -np.cross(w1,q1)
	v2 = -np.cross(w2,q2)
	v3 = -np.cross(w3,q3)
	v4 = -np.cross(w4,q4)
	v5 = -np.cross(w5,q5)
	v6 = -np.cross(w6,q6)
    
	S[:, 0] = np.concatenate((w1, v1))
	S[:, 1] = np.concatenate((w2, v2))
	S[:, 2] = np.concatenate((w3, v3))
	S[:, 3] = np.concatenate((w4, v4))
	S[:, 4] = np.concatenate((w5, v5))
	S[:, 5] = np.concatenate((w6, v6))

	# ==============================================================#
	return M, S

	# =================== Screw_matrix Fuction ====================#
def screw_matrix(screw_axis):
	omega = screw_axis[:3]  # Angular velocity
	v = screw_axis[3:]      # Linear velocity
	omega_hat = np.array([[0, -omega[2], omega[1]],
                          [omega[2], 0, -omega[0]],
                          [-omega[1], omega[0], 0]])
	s = np.zeros((4, 4))
	s[:3, :3] = omega_hat
	s[:3, 3] = v
	return s


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# =========== Implement joint angle to encoder expressions here ===========
	print("Forward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS() #call Get MS
    
	t = np.eye(4)
	theta = np.array([theta1-PI/2, theta2, theta3, theta4+PI/2, theta5, theta6])
    
	for i in range(len(theta)):
		ScrewM=screw_matrix(S[:, i])
		exp_theta=expm(theta[i] * ScrewM)
		t = np.dot(t, exp_theta) #t is T12, 13, ...16
        
	T=np.dot(t,M) #T06
	# ==============================================================#
	
	print(str(T) + "\n")
	return T

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	l01 = 0.152
	l02 = 0.120
	l03 = 0.244
	l04 = 0.093
	l05 = 0.213
	l06 = 0.104
	l07 = 0.085
	l08 = 0.092
	l09 = 0
	#l09=0。0535
	l10 = 0.070

	yaw = np.radians(yaw_WgripDegree)
	#yaw=yaw_WgripDegree
	xgrip = -yWgrip
	ygrip = xWgrip
	zgrip = zWgrip

	xcen = xgrip - l09 * np.cos(yaw)
	ycen = ygrip - l09 * np.sin(yaw)
	zcen = zgrip

	# theta1
	thetas[0] = np.arctan2(ycen,xcen)-np.arcsin((l02 - l04 + l06) / np.sqrt(xcen ** 2 + ycen ** 2))        # Default value Need to Change

	# theta6
	thetas[5] = thetas[0]+PI/2-yaw     # Default value Need to Change
 
	x3end = xcen-l07*np.cos(thetas[0])+(l06+0.027)*np.sin(thetas[0])
	y3end = ycen-l07*np.sin(thetas[0])-(l06+0.027)*np.cos(thetas[0])
	z3end = zcen+l08+l10

	R = (x3end**2+y3end**2+(z3end-l01)**2)**0.5

	alpha=np.arccos((R**2+l03**2-l05**2)/2/R/l03)
	beta=np.arctan2(z3end-l01,np.sqrt(x3end**2+y3end**2))

	thetas[1]= -(alpha+beta)     # Default value Need to Change
	thetas[2]= PI-np.arccos((l03**2+l05**2-R**2)/2/l03/l05)     # Default value Need to Change
	thetas[3]= -(thetas[1]+thetas[2])     # Default value Need to Change
	 # Default value Need to Change
	thetas[4]=-PI/2      # Default value Need to Change

	thetas[0] = thetas[0] + PI/2
	thetas[3] = thetas[3] - PI/2

	print("theta1 to theta6: " + str(thetas) + "\n")

	return thetas
