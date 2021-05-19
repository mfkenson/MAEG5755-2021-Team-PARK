#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import DeleteModel


def main():
    rospy.init_node('delete_model', anonymous=True)
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        resp = del_model_prox( "wood_block_1" )
        print(resp)
        rospy.sleep(2)

        del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        resp = del_model_prox( "wood_block_2" )
        print(resp)
        rospy.sleep(2)
        del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        resp = del_model_prox( "wood_block_3" )
        print(resp)
        rospy.sleep(2)

    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass