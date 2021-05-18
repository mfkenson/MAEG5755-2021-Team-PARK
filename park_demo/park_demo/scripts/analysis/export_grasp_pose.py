#!/usr/bin/env python3.6
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import pandas as pd
import cv2


from autolab_core import YamlConfig, Logger
from perception import (BinaryImage, CameraIntrinsics, ColorImage, DepthImage,
                        RgbdImage)
from visualization import Visualizer2D as vis

from gqcnn.grasping import (RobustGraspingPolicy,
                            CrossEntropyRobustGraspingPolicy, RgbdImageState,
                            FullyConvolutionalGraspingPolicyParallelJaw,
                            FullyConvolutionalGraspingPolicySuction)
from gqcnn.utils import GripperMode

from geometry_msgs.msg import PoseStamped
import baxter_interface
from math import pi
import tf

def process_image_to_array(ros_image):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    ret = np.array(img, dtype=np.float32)
    if len(ret.shape) == 3: #3 channels? only need one
        ret , _, _ = cv2.split(ret)
    return ret/1000.0 #meter
    
def perform_inference(depth_arr):
    model_dir = '/home/developr/Documents/dexnet_baxter/models/GQCNN-4.0-PJ'
    model_params = '/home/developr/Documents/dexnet_baxter/dexnet_deps/gqcnn/cfg/examples/replication/dex-net_4.0_pj.yaml'
    camera_intr_filename = '/home/developr/5755_ws/src/team-park/park_dexnet/config/camera_d435_gazebo.intr'
    camera_intr = CameraIntrinsics.load(camera_intr_filename)
    config = YamlConfig(model_params)
    policy_config = config["policy"]
    policy_config["metric"]["gqcnn_model"] = model_dir
    #inpaint_rescale_factor = config["inpaint_rescale_factor"]
    depth_im = DepthImage(depth_arr, frame=camera_intr.frame)
    color_im = ColorImage(np.zeros([depth_im.height, depth_im.width,
                                        3]).astype(np.uint8),
                            frame=camera_intr.frame)
    #no segmask; all ones
    segmask = BinaryImage(255 *
                          np.ones(depth_im.shape).astype(np.uint8),
                          frame=color_im.frame)
    rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)
    state = RgbdImageState(rgbd_im, camera_intr, segmask=segmask)
    policy = CrossEntropyRobustGraspingPolicy(policy_config)
    try:
        grasp_candidates, q_values = policy.action_set(state)
    except Exception as e:
        grasp_candidates = []
    candidate_array = []
    for i in range(len(grasp_candidates)):
        # Position: [x y z] # Orientation: [qw qx qy qz]
        candidate_array.append(np.array([
            #grasp_candidates[i].pose().to_frame,
                np.round(grasp_candidates[i].pose().position, 6),
                np.round(grasp_candidates[i].pose().quaternion,6),
                q_values[i],
                grasp_candidates[i].center.vector, #center is a type of autolab_core.Point
                grasp_candidates[i].angle,
                grasp_candidates[i].depth,
                grasp_candidates[i].width,
                grasp_candidates[i].contact_points,
                grasp_candidates[i].contact_normals,
                grasp_candidates[i].axis,
                grasp_candidates[i].approach_angle,
                grasp_candidates[i].width_px,
                grasp_candidates[i].endpoints,
                grasp_candidates[i].feature_vec])
        )
    np.save('grasp_pose', candidate_array)
    if len(candidate_array)==0:
        return None
    result = np.array(candidate_array)
    return result
    #df = pd.DataFrame(result, columns =['position', 'quaternion', 'q_value'])
    #sorted_arr = df.sort_values('q_value', ascending=False).to_numpy()
    #return sorted_arr

    
def main():
    rospy.init_node('export_grasp_pose', anonymous=True)
    #topic_name = '/kinect/depth/image_raw'
    topic_name = '/d435/depth/image_raw'

    topic_name = rospy.get_param('~image_topic', topic_name)
    msg = rospy.wait_for_message(topic_name, Image)
    rospy.loginfo('Received an image')
    arr = process_image_to_array(msg)
    poses_candidates = perform_inference(arr)
    if poses_candidates is None:
        rospy.logwarn("No candidates found")
    else:
        rospy.loginfo("Best candidate")
        print(poses_candidates[0])
if __name__ == '__main__':
    main()
