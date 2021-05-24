#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2

def export_image(ros_image, file_name):
    bridge = CvBridge()
    dep = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    dep = np.array(dep, dtype=np.float32)  # mm
    mask = 255*np.ones(dep.shape, dtype=np.uint8)
    np.save(file_name, mask)
    cv2.imwrite(file_name.replace('.npy', ".jpeg"), mask)

def main():
    rospy.init_node('export_image', anonymous=True)

    topic_name = '/d435/depth/image_rect_raw'#real env
    #topic_name = '/d435/depth/image_raw'

    topic_name = rospy.get_param('~image_topic', topic_name)
    rospy.loginfo('wait for topic:')
    rospy.loginfo(topic_name)
    file_name = 'depth_roi_mask_uint8.npy'
    msg = rospy.wait_for_message(topic_name, Image)
    export_image(msg, file_name)


if __name__ == '__main__':
    main()
