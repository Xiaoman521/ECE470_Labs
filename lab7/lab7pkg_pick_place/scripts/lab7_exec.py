#!/usr/bin/env python

'''

lab7pkg_pick_place/lab7_exec.py

@brief: main file to execute the pick and place task

@author: Songjie Xiao
@date: Monday 2023/03/23

'''

# ckcamera api
import sys
sys.path.append("../utils/script/")
from ckcamera import *

from lab7_func import *
from lab7_ur3e import *
from lab7_img import *

def main():

	# Initialize ROS node
	rospy.init_node('lab7_node')

	# Initialize CKCamera
	# camera = CKCamera()
	# camera.init()

	# 1. save first calibration image, enter 's' to save image with an input of image_name
	# camera.display()
	# exit()
	# 2. Free move the robot to obtain the robot coordinate (x,y) of first block
	# TODO: get input of robot coordinate (x,y) of first block
	center_robot1 = ()
	# 3. save second calibration image, enter 's' to save image with an input of image_name
	# camera.display()
	# 4. Free move the robot to obtain the robot coordinate (x,y) of second block
	# TODO: get input of robot coordinate (x,y) of second block
	center_robot2 = ()

	# 5. save snapshot image, randomly place at least 5 blocks
	# camera.display()

	# uninit CKCamera
	# camera.uninit()

	# TODO: complete the coordinate transformation function in lab7_img.py
	# 6. do Image Processing and coordinate transformation to obtain the robot coordinate (x,y) of each block
	center_values, shape, theta = lab_imgproc(center_robot1, center_robot2)
	
	# exit()
	############## Your Code Start Here ############## 	
	# TODO: main execution of pick and place task

	# initialize UR3e functions
	ur3e = UR3e()
	print("Please Press 'Start' button on the Teach Pendant to continue ...")

	# wait for the 'Start' button on the Teach Pendant
	time.sleep(2)
	center_values, shapes, thetas = lab_imgproc(center_robot1, center_robot2)
	home_pos = lab_invk()
	
	# call inverse kinematics to compute joint values according to robot coordinate
	# z values is assigned by users
	dest_pos_rec = lab_invk()
	dest_pos_eli = lab_invk()

	# do the move_arm and gripper operation to pick and place the block
	# ur3e.move(dest_pos)
	# ur3e.gripper(True)


	ur3e.move_arm(home_pos)
	print("1\n")

	print(center_values)
	print(shapes)
	print(thetas)
	# exit()
	for center,shape,theta in zip(center_values, shapes, thetas):
		print("2\n")
		print(center_values)
		x = center[0]
		y = center[1]
		start_pos_1 = lab_invk(x, y, 0.07, 0)
		start_pos_2 = lab_invk(x, y, 0.15, 0)
		print(start_pos_1)

		print("3\n")
		ur3e.move_arm(start_pos_2)
		ur3e.move_arm(start_pos_1)
		ur3e.gripper(True)
		ur3e.move_arm(start_pos_2)

		if(shape == 0):
			print(0)
			ur3e.move_arm(dest_pos_rec)
			ur3e.gripper(False)
		else:
			print(1)
			ur3e.move_arm(dest_pos_eli)
			ur3e.gripper(False)

		print("4\n")
		ur3e.move_arm(home_pos)


	pass

	############### Your Code End Here ###############

	rospy.loginfo("Finish Pick and place task!")

if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass
