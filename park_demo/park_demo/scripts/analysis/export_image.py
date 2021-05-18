#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


def export_image(ros_image, file_name):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    arr = np.array(img, dtype=np.float32)  # mm
    np.save(file_name, arr)
    rospy.loginfo('passthrough saved file:')
    rospy.loginfo(file_name)

def main():
    rospy.init_node('export_image', anonymous=True)
    topic_name = '/kinect/rgb/image_raw'
    topic_name = '/kinect/depth/image_raw'
    topic_name = '/d435/depth/image_raw'

    topic_name = rospy.get_param('~image_topic', topic_name)
    rospy.loginfo('wait for topic:')
    rospy.loginfo(topic_name)
    file_name = topic_name.replace('/', '_') + '.npy'
    msg = rospy.wait_for_message(topic_name, Image)
    export_image(msg, file_name)


if __name__ == '__main__':
    main()
