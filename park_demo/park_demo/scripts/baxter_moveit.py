#!/usr/bin/env python3

import rospy
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped
import baxter_interface
from math import pi
import tf

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

def ready_pose(arm='left'):
    frame_id = 'world'
    position = [0.5, 0.5, 1.3]
    if arm == 'right':
        position = [0.5, -0.5, 1.3]
    orientation = [ 1,0,0,0] #face down
    return fill_goal_message(frame_id, position, orientation)

def main():
    rospy.init_node('baxter_moveit_moveToPose', anonymous=True)
    p = PlanningSceneInterface("base")
    g = MoveGroupInterface("both_arms", "base")
    gr = MoveGroupInterface("right_arm", "base")
    gl = MoveGroupInterface("left_arm", "base")
    leftgripper = baxter_interface.Gripper('left')
    
    #gl.moveToPose(ready_pose(), "left_gripper", plan_only=False)
    gr.moveToPose(ready_pose(arm='right'), "right_gripper", plan_only=False)

if __name__ == '__main__':
    main()
