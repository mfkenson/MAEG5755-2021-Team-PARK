#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import pandas as pd
import cv2
import baxter_interface

from moveit_msgs.msg import MoveItErrorCodes
import time
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped
import baxter_interface
from math import pi
import tf
import moveit_commander

def fill_goal_message_array(frame_id, position, orientation):
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


def fill_goal_message(frame_id, position, orientation):
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = rospy.Time.now()
    goal.pose.position = position
    goal.pose.orientation = orientation
    return goal

def ready_pose(arm='left'):
    frame_id = 'world'
    position = [0.5, 0.5, 1.3]
    if arm == 'right':
        position = [0.5, -0.5, 1.3]
    orientation = [ 1,0,0,0] #face down
    return fill_goal_message_array(frame_id, position, orientation)


def pre_g_pose(target_pose):
    #
    orientation = None
    #orientation = [1, 0, 0, 0]  # face down
    z_offset = None
    #z_offset = 0.15
    goal = target_pose
    if z_offset is not None:
        goal.pose.position.z = goal.pose.position.z + z_offset
    if orientation is not None:
        goal.pose.orientation.x = orientation[0]
        goal.pose.orientation.y = orientation[1]
        goal.pose.orientation.z = orientation[2]
        goal.pose.orientation.w = orientation[3]
    return goal


def load_candidates():
    return np.load('_d435_dexnet_pose_17.npy', allow_pickle=True)


def main():
    rospy.init_node('moveit_gogogo', anonymous=True)
    #topic_name = '/kinect/depth/image_raw'

    camera_frame = 'd435_depth_optical_frame'
    world_frame = 'world'
    listener = tf.TransformListener()
    listener.waitForTransform(world_frame, camera_frame, rospy.Time.now(), rospy.Duration(10.0))
    rospy.loginfo('Received a Transform')


    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("both_arms")

    left_current_pose = group.get_current_pose(end_effector_link='left_gripper')
    right_current_pose = group.get_current_pose(end_effector_link='right_gripper')
    #print('left_current_pose', left_current_pose)
    #print('right_current_pose', right_current_pose)
    pre_g = None
    gr = MoveGroupInterface("right_arm", "base")
    gl = MoveGroupInterface("left_arm", "base")
   # gl.moveToPose(ready_pose(), "left_gripper")
   # rospy.sleep(10)
    poses_candidates = load_candidates()

    if poses_candidates is None:
        rospy.logwarn("No candidates found")
    else:

        attempt_counter = 17
        leftgripper = baxter_interface.Gripper('left')
        rightgripper = baxter_interface.Gripper('right')

        leftgripper.open()
        rightgripper.open()

        current_candidate = poses_candidates[attempt_counter]
        rospy.loginfo("Current attempt #: " + str(attempt_counter))
        #print(current_candidate)
        pose_camera_frame = fill_goal_message_array(camera_frame, current_candidate[0], current_candidate[1])
        pose_world_frame = listener.transformPose('world', pose_camera_frame)
        print(pose_world_frame)
        result = gl.moveToPose(pre_g_pose(pose_world_frame), "left_gripper", planning_time=5, plan_only=False)
        print(result)
        if result.error_code.val == -1:
            attempt_counter = attempt_counter + 1
        else:
            pass
        rospy.sleep(5)


if __name__ == '__main__':
    main()
