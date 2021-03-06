#!/usr/bin/env python3


import roslaunch
import rospy
import os

home_path = os.environ['HOME']
rospy.init_node('spawn_stuff', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, [ home_path+"/5755_ws/src/team-park/park_simulation/park_gazebo/add_coke.launch"])
launch.start()
rospy.loginfo("started coke")
rospy.sleep(5)