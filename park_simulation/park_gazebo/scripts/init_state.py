#!/usr/bin/env python3

import rospy 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


def main():
    rospy.init_node('set_pose')
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    #-----------------------------------------------------------------------------
    resp = set_state(reset_table_msg())
    rospy.sleep(3)
    # -----------------------------------------------------------------------------
    resp = set_state(reset_coke_msg())
    rospy.sleep(3)
# -----------------------------------------------------------------------------
    resp = set_state(reset_robot_msg())
    rospy.sleep(3)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass