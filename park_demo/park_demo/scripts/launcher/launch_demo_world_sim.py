#!/usr/bin/env python3


import roslaunch
import rospy
import os

home_path = os.environ['HOME']
rospy.init_node('launch_demo_world', anonymous=True)
# --------------------EMPTY WORLD---------------------
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/opt/ros/noetic/share/gazebo_ros/launch/empty_world.launch"])
launch.start()
rospy.loginfo("started empty world")
rospy.sleep(10)
# ---------------------BAXTER--------------------
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, [home_path+"/5755_ws/src/team-park/park_simulation/park_gazebo/baxter_on_pedestal_w_realsense.launch"])
launch.start()
rospy.loginfo("started baxter + realsense")
rospy.sleep(10)


rospy.sleep(36000) #10 hours later......
