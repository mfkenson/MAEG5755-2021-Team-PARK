#!/usr/bin/env python

import sys, pickle
import rospy
from moveit_python.planning_scene_interface import PlanningScene

if __name__ == "__main__":
    rospy.init_node("load_planning_scene", anonymous=True)
    pub = rospy.Publisher('planning_scene',
                          PlanningScene,
                          queue_size=10,
                          latch=True)

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "scene.saved"

    scene = pickle.load(open(filename, "rb"))
    scene.scene.is_diff = True
    pub.publish(scene.scene)
    rospy.sleep(5)
