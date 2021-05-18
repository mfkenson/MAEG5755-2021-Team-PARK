#!/usr/bin/env python3

import rospy
import baxter_interface

def main():
    rospy.init_node('baxter_grippers_close', anonymous=True)

    leftgripper = baxter_interface.Gripper('left')
    rightgripper = baxter_interface.Gripper('right')

    leftgripper.calibrate()
    rightgripper.calibrate()

    leftgripper.close()
    rightgripper.close()

    rospy.sleep(3)


if __name__ == '__main__':
    main()
