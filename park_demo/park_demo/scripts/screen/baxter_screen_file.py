#!/usr/bin/env python
import rospy
import sys
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

xdisplay_topic = "/robot/xdisplay"

def start_node(filename='cuhk.png'):
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')
    img = cv2.imread(filename)
    bridge = CvBridge()
    imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")
    pub = rospy.Publisher(xdisplay_topic, Image, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(imgMsg)
        rospy.Rate(1.0).sleep()  # 1 Hz

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass