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
    norm_image = cv2.normalize(dep, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    roi = cv2.selectROI(norm_image)
    roi_cropped = norm_image[int(roi[1]):int(roi[1] + roi[3]), int(roi[0]):int(roi[0] + roi[2])]
    mask = np.zeros(dep.shape, dtype=np.uint8)
    (x, y, w, h) = roi
    mask[y:y + h, x:x + w] = 255
    cv2.imshow("mask", mask)
    cv2.imshow("crop", roi_cropped)
    np.save(file_name, mask)
    cv2.imwrite(file_name.replace('.npy', ".jpeg"), mask)
    # hold window
    cv2.waitKey(0)

def main():
    rospy.init_node('export_image', anonymous=True)

    #topic_name = '/d435/depth/image_rect_raw'#real env
    topic_name = '/d435/depth/image_raw'

    topic_name = rospy.get_param('~image_topic', topic_name)
    rospy.loginfo('wait for topic:')
    rospy.loginfo(topic_name)
    file_name = 'depth_roi_mask_uint8.npy'
    msg = rospy.wait_for_message(topic_name, Image)
    export_image(msg, file_name)


if __name__ == '__main__':
    main()
