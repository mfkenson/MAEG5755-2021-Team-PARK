#!/usr/bin/env python3

import rospy
import moveit_commander
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped
import baxter_interface
from math import pi
import tf
import copy
import math
import numpy as np
def fill_goal_message(frame_id, position, orientation):
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = position[0]
    goal.pose.position.y = position[1]
    goal.pose.position.z = position[2] 
    goal.pose.orientation.x = orientation[0]
    goal.pose.orientation.y = orientation[1]
    goal.pose.orientation.z = orientation[2]
    goal.pose.orientation.w = orientation[3]
    return goal


def move_xyz_relative(group, z=0, y=0):
    #moveit commander group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z += z
    wpose.position.y += y
    waypoints.append(copy.deepcopy(wpose))
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.01,  # eef_step
        0.0)  # jump_threshold
    # Note: We are just planning, not asking move_group to actually move the robot yet:

    plan = group.retime_trajectory(group.get_current_state(),
        plan, velocity_scaling_factor=0.2, acceleration_scaling_factor=0.2,
        algorithm="time_optimal_trajectory_generation")
    return plan, fraction


def main():
    rospy.init_node('baxter_moveit_moveToPose', anonymous=True)


    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("left_arm")

    left_current_pose = group.get_current_pose(end_effector_link='left_gripper')

    leftgripper = baxter_interface.Gripper('left')
    rightgripper = baxter_interface.Gripper('right')
    rospy.sleep(1)
    leftgripper.open(block=True, timeout=1.0)

    rospy.sleep(1)
    plan, _ = move_xyz_relative(group, z=-0.15)
    group.execute(plan, wait=True)
    rospy.sleep(1)
    leftgripper.close(block=True, timeout=1.0)
    rospy.sleep(1)
    plan, _ = move_xyz_relative(group, z=0.15)
    group.execute(plan, wait=True)
    rospy.sleep(1)



    wpose = group.get_current_pose().pose
    target_y = 0.6 - wpose.position.y
    plan, _ = move_xyz_relative(group, z=0, y=target_y)
    group.execute(plan, wait=True)
    rospy.sleep(1)
    plan, _ = move_xyz_relative(group, z=-0.15)
    group.execute(plan, wait=True)
    leftgripper.open(block=True, timeout=1.0)
    rospy.sleep(1)
    plan, _ = move_xyz_relative(group, z=0.3)
    group.execute(plan, wait=True)
    '''
    #plan, _ = move_xyz_relative(group, y=-0.3)
    #group.execute(plan, wait=True)
    print('OK')

    
     
    '''

if __name__ == '__main__':
    main()
