#!/usr/bin/env python3


import roslaunch
import rospy
import os

home_path = os.environ['HOME']
rospy.init_node('launch_moveit_plan', anonymous=True)

# ----------------------moveit-------------------
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, [home_path+"/5755_ws/src/team-park/park_moveit/baxter_moveit_tutorial/launch/moveit_init.launch"])
launch.start()
rospy.loginfo("started baxter moveit")
rospy.sleep(10)
# -----------------------------------------

rospy.sleep(36000) #10 hours later......
