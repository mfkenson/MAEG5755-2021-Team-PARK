#!/usr/bin/env python3

import baxter_interface
import rospy

def main():
    rospy.init_node("baxter_disable", anonymous=True)
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    print("Getting robot state... ")
    _init_state = rs.state().enabled
    print("Disabling robot... ")
    rs.disable()
    rospy.sleep(3)


if __name__ == '__main__':
    main()
