#!/usr/bin/env python3

import rospy
import moveit_commander
import baxter_interface
import numpy as np
import math


def deg2rad(deg):
    return deg*math.pi/180.0

def main():
    rospy.init_node('baxter_get_joint_values', anonymous=True)
    joint_state_topic = ['joint_states:=/robot/joint_states'] #essential
    moveit_commander.roscpp_initialize(joint_state_topic)
    group = moveit_commander.MoveGroupCommander("both_arms")
    gl = moveit_commander.MoveGroupCommander("left_arm")
    gr = moveit_commander.MoveGroupCommander("right_arm")

    leftgripper = baxter_interface.Gripper('left')
    rightgripper = baxter_interface.Gripper('right')

    left_current_joints_pos = gl.get_current_joint_values() #radian
    right_current_joints_pos = gr.get_current_joint_values() #radian
    print('--------------------------------------------------------------------------')
    print('left_current_joints_pos: ',np.array(left_current_joints_pos)*180/math.pi)
    print('right_current_joints_pos: ',np.array(right_current_joints_pos)*180/math.pi)
    leftgripper.calibrate()
    leftgripper.open()
    rospy.sleep(5)
    '''
    # no guarded; use with caution
        joint_goal = gl.get_current_joint_values()
        joint_goal[0] = deg2rad(-45)
        joint_goal[1] = deg2rad(-45)
        joint_goal[2] = 0.0
        joint_goal[3] = deg2rad(45)
        joint_goal[4] = 0.0
        joint_goal[5] = deg2rad(90)
        joint_goal[6] = 0.0
        print(type(joint_goal))
        gl.go(joint_goal, wait=True)
        gl.stop()
    
    
        joint_goal = gr.get_current_joint_values()
        joint_goal[0] = deg2rad(45)
        joint_goal[1] = deg2rad(-45)
        joint_goal[2] = 0.0
        joint_goal[3] = deg2rad(45)
        joint_goal[4] = 0.0
        joint_goal[5] = deg2rad(90)
        joint_goal[6] = 0.0
        print(type(joint_goal))
        gr.go(joint_goal, wait=True)
        gr.stop()
    '''


if __name__ == '__main__':
    main()
