#!/usr/bin/env python3

import rospy
import moveit_commander


def get_baxter_pose():
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('baxter_get_ee_pose', anonymous=True)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("left_arm")

    left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    right_current_pose = group.get_current_pose(end_effector_link='right_gripper').pose
    print('left_current_pose', left_current_pose)
    print('right_current_pose', right_current_pose)
    rospy.sleep(3)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    get_baxter_pose()
