#!/usr/bin/env python3


import roslaunch
import rospy
import os

home_path = os.environ['HOME']
rospy.init_node('launch_demo_world', anonymous=True)
#---------------------REAL ENV-------------------------

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, [home_path + '/5755_ws/src/learning_ros_external_pkgs_noetic/baxter_simulator/baxter_gazebo/launch'])
launch.start()
rospy.loginfo("started Baxter world")
rospy.sleep(10)
rospy.sleep(36000) #10 hours later......
