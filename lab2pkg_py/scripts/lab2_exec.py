#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
from unicodedata import digit
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([180, -90, 90, -90, -90, 90])

# Hanoi tower location 1
Q11 = [142.16*pi/180.0, -64.32*pi/180.0, 145.59*pi/180.0, -170.32*pi/180.0, -90*pi/180.0, 53.26*pi/180.0]
Q12 = [142.21*pi/180.0, -77.5*pi/180.0, 143.96*pi/180.0, -155.52*pi/180.0, -90*pi/180.0, 53.18*pi/180.0]
Q13 = [142.27*pi/180.0, -88.17*pi/180.0, 140.60*pi/180.0, -141.49*pi/180.0, -90*pi/180.0, 53.09*pi/180.0]
# Hanoi tower location 2
Q21 = [171.89*pi/180.0, -64.72*pi/180.0, 142.41*pi/180.0, -166.32*pi/180.0, -90*pi/180.0, 82.95*pi/180.0]
Q22 = [171.93*pi/180.0, -76.24*pi/180.0, 140.83*pi/180.0, -153.21*pi/180.0, -90*pi/180.0, 82.87*pi/180.0]
Q23 = [171.98*pi/180.0, -86.14*pi/180.0, 137.57*pi/180.0, -140.05*pi/180.0, -90*pi/180.0, 82.79*pi/180.0]
# Hanoi tower location 3
Q31 = [198.96*pi/180.0, -57.81*pi/180.0, 122.85*pi/180.0, -153.57*pi/180.0, -90*pi/180.0, 109.91*pi/180.0]
Q32 = [198.98*pi/180.0, -65.59*pi/180.0, 121.66*pi/180.0, -144.60*pi/180.0, -90*pi/180.0, 109.84*pi/180.0]
Q33 = [199.01*pi/180.0, -72.54*pi/180.0, 119.15*pi/180.0, -135.13*pi/180.0, -90*pi/180.0, 109.76*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]

############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_callback(msg):
    global digital_in_0
    global analog_in_0
    
    digital_in_0 = msg.DIGIN
    analog_in_0 = msg.AIN0


############### Your Code End Here ###############


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

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


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
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############
# done?

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global analog_in_0

    start_pos = Q[start_loc][start_height]
    end_pos = Q[end_loc][end_height]
    vel = 2.0
    acc = 2.0
    error = 0

    move_arm(pub_cmd, loop_rate, start_pos, vel, acc)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)
    if (analog_in_0 < 2.0):
        print("Block missing in expected position!")
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        sys.exit()

    
    move_arm(pub_cmd, loop_rate, home, vel, acc)

    

    move_arm(pub_cmd, loop_rate, end_pos, vel, acc)
    gripper(pub_cmd, loop_rate, suction_off)
    move_arm(pub_cmd, loop_rate, home, vel, acc)
    
    return error

############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input
    # done

    input_done = 0
    tower_start_pos = 0
    tower_end_pos = 0

    while(not input_done):
        input_string = raw_input("Enter the starting position of the tower <Either 1, 2, or 3; or 0 to quit> ")
        print("You entered position " + input_string + "\n")

        if (input_string == "0"):
            print("Quitting... ")
            sys.exit()
        elif (input_string == "1" or input_string == "2" or input_string == "3"):
            tower_start_pos = int(input_string)
            input_done = 1
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    input_done = 0
    while(not input_done):
        input_string = raw_input("Enter the ending position of the tower <Either 1, 2, or 3; or 0 to quit> ")
        print("You entered position " + input_string + "\n")

        if (input_string == "0"):
            print("Quitting... ")
            sys.exit()
        elif (input_string == "1" or input_string == "2" or input_string == "3"):
            tower_end_pos = int(input_string)
            input_done = 1
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    if (tower_start_pos == tower_end_pos):
        print("Done!")
        sys.exit()

    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    pos1 = 0
    pos2 = 1
    pos3 = 2
    bot = 0
    mid = 1
    top = 2

    move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    if (tower_start_pos == 1):
        pos1 = 0
        if (tower_end_pos == 2):
            pos2 = 1
            pos3 = 2
        else:
            pos2 = 2
            pos3 = 1
    elif (tower_start_pos == 2):
        pos1 = 1
        if (tower_end_pos == 1):
            pos2 = 0
            pos3 = 2
        else:
            pos2 = 2
            pos3 = 0
    elif (tower_start_pos == 3):
        pos1 = 2
        if (tower_end_pos == 1):
            pos2 = 0
            pos3 = 1
        else:
            pos2 = 1
            pos3 = 0
    
    move_block(pub_command, loop_rate, pos1, top, pos2, bot)
    move_block(pub_command, loop_rate, pos1, mid, pos3, bot)
    move_block(pub_command, loop_rate, pos2, bot, pos3, mid)
    move_block(pub_command, loop_rate, pos1, bot, pos2, bot)
    move_block(pub_command, loop_rate, pos3, mid, pos1, bot)
    move_block(pub_command, loop_rate, pos3, bot, pos2, mid)
    move_block(pub_command, loop_rate, pos1, bot, pos2, top)

    # while(tower_start_pos > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     tower_start_pos = tower_start_pos - 1

    # gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
