#!/usr/bin/env python3

import rospy 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


def fill_in_model_state_msg(model_name, position, orientation):
    state_msg = ModelState()
    state_msg.model_name = model_name
    if position is None:
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
    else:
        state_msg.pose.position.x = position['x']
        state_msg.pose.position.y = position['y']
        state_msg.pose.position.z = position['z']

    if orientation is None:
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
    else:
        state_msg.pose.orientation.x = orientation['x']
        state_msg.pose.orientation.y = orientation['y']
        state_msg.pose.orientation.z = orientation['z']
        state_msg.pose.orientation.w = orientation['w']
    return state_msg



def reset_robot_msg():
    position = dict()
    position['x'] = 0.0
    position['y'] = 0.0
    position['z'] = 0.0
    return fill_in_model_state_msg('baxter_on_pedestal_realsense', position, None)


def reset_coke_msg():
    position = dict()
    position['x'] = 0.77
    position['y'] = 0.15
    position['z'] = 0.77
    return fill_in_model_state_msg('coke_can', position, None)


def reset_table_msg(x=1.0):
    position = dict()

    position['x'] = x
    position['y'] = 0
    position['z'] = 0

    return fill_in_model_state_msg('cafe_table', position, None)


def main():
    rospy.init_node('reset_pose', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    #-----------------------------------------------------------------------------
    # -----------------------------------------------------------------------------
    resp = set_state(reset_coke_msg())
    rospy.sleep(3)
# -----------------------------------------------------------------------------



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass