#!/usr/bin/env python3

import rospy

# Because of transformations
import tf2_ros
import geometry_msgs.msg
from rospy import Time


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
        msg = manual_transform('base', 'torso', [0,0,0], [0,0,0,1])
        br.sendTransform(msg)
        msg = manual_transform('world', 'base', [0,0,0], [0,0,0,1])
        br.sendTransform(msg)
        #eye to hand
        #0.892837 -0.242724 0.42847   0.93645 -0.00247109 -0.0409275 -0.348398
        #msg = manual_transform('torso', 'd435_color_optical_frame', [0.892837, -0.257724, 0.42847], [0.93645,-0.00247109,-0.0409275,-0.348398])
        br.sendTransform(msg)
        rospy.sleep(0.01)
