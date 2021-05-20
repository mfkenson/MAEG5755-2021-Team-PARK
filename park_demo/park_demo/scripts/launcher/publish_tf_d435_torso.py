#!/usr/bin/env python3

import rospy

# Because of transformations
import tf2_ros
import geometry_msgs.msg


def manual_transform(parent_frame_id, child_frame_id, pos, ori):
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = pos[0]
    t.transform.translation.y = pos[1]
    t.transform.translation.z = pos[2]
    t.transform.rotation.x = ori[0]
    t.transform.rotation.y = ori[1]
    t.transform.rotation.z = ori[2]
    t.transform.rotation.w = ori[3]
    return t


if __name__ == '__main__':
    rospy.init_node('tf2_park_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    while True:
        msg = manual_transform('torso', 'd435_color_optical_frame', [1.1101, -0.2717, 0.4262], [-0.9342, 0.0051, 0.0376, 0.3548])
        br.sendTransform(msg)
        rospy.sleep(0.1)
