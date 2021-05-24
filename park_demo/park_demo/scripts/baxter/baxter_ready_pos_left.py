#!/usr/bin/env python3

import rospy
import moveit_commander
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_python.planning_scene_interface import (GetPlanningScene,
                                                    PlanningSceneComponents)
import baxter_interface
import numpy as np
import math


def main():
    rospy.init_node('baxter_get_ready_right', anonymous=True)
    joint_state_topic = ['joint_states:=/robot/joint_states']

    jts_right = ['right_s0', 'right_s1', 'right_e0', 'right_e1','right_w0', 'right_w1', 'right_w2']
    jts_left = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

    rospy.loginfo("Waiting for get_planning_scene")
    rospy.wait_for_service("get_planning_scene")
    service = rospy.ServiceProxy("get_planning_scene", GetPlanningScene)
    #leftgripper = baxter_interface.Gripper('left')
    #rightgripper = baxter_interface.Gripper('right')
    gl = MoveGroupInterface("left_arm", "base")

    try:
        req = PlanningSceneComponents()
        req.components = PlanningSceneComponents.WORLD_OBJECT_NAMES + \
                         PlanningSceneComponents.WORLD_OBJECT_GEOMETRY + \
                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        scene = service(req)
        #ready_joints_left = np.array([-45.0, -45.0, 0, 45, 0.0, 90.0, 0.0]) * math.pi / 180.0 #zombie
        ready_joints_left = np.array([
            -22.03857422, -24.38964844, -85.73730469,   58.07373047, -115.62011719,-89.97802734, -44.09912109
        ]
        ) * math.pi / 180.0
        result = gl.moveToJointPosition(jts_left, ready_joints_left, plan_only=False, wait=True, max_velocity_scaling_factor=0.3)
        print(result)

    except rospy.ServiceException as e:
        print("Failed to get planning scene: %s" % e)
        exit(-1)





if __name__ == '__main__':
    main()
