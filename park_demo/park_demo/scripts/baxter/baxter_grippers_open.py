#!/usr/bin/env python3

import rospy
import baxter_interface

def main():
    rospy.init_node('baxter_grippers_open', anonymous=True)
    rospy.sleep(1)
    leftgripper = baxter_interface.Gripper('left')
    rightgripper = baxter_interface.Gripper('right')
    rospy.sleep(1)

    leftgripper.open(block=True, timeout=1.0)
    rightgripper.open(block=True, timeout=1.0)

    rospy.sleep(3)


if __name__ == '__main__':
    main()
