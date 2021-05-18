#!/usr/bin/env python

import sys, pickle
import rospy
from moveit_python.planning_scene_interface import (GetPlanningScene,
                                                    PlanningSceneComponents)

if __name__ == "__main__":
    rospy.init_node("dump_planning_scene", anonymous=True)

    rospy.loginfo("Waiting for get_planning_scene")
    rospy.wait_for_service("get_planning_scene")
    service = rospy.ServiceProxy("get_planning_scene", GetPlanningScene)
    try:
        req = PlanningSceneComponents()
        req.components = PlanningSceneComponents.WORLD_OBJECT_NAMES + \
                         PlanningSceneComponents.WORLD_OBJECT_GEOMETRY  + \
                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        scene = service(req)
    except rospy.ServiceException as e:
        print("Failed to get planning scene: %s" % e)
        exit(-1)

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "scene.saved"

    print("dumping")
    print(scene)
    print("to %s" % filename)

    pickle.dump(scene, open(filename, "wb"))
