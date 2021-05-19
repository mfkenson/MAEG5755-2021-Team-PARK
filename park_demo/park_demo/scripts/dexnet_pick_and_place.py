#!/usr/bin/env python3.6
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import pandas as pd
import cv2
import baxter_interface
import math

from autolab_core import YamlConfig, Logger
from perception import (BinaryImage, CameraIntrinsics, ColorImage, DepthImage,
                        RgbdImage)
from visualization import Visualizer2D as vis

from gqcnn.grasping import (RobustGraspingPolicy,
                            CrossEntropyRobustGraspingPolicy, RgbdImageState,
                            FullyConvolutionalGraspingPolicyParallelJaw,
                            FullyConvolutionalGraspingPolicySuction)
from gqcnn.utils import GripperMode
from moveit_msgs.msg import MoveItErrorCodes
import time
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from geometry_msgs.msg import PoseStamped
import baxter_interface
from math import pi
import tf
import moveit_commander


def process_image_to_array(ros_image):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    ret = np.array(img, dtype=np.float32)
    if len(ret.shape) == 3: #3 channels? only need one
        ret , _, _ = cv2.split(ret)
    return ret/1000.0 #meter
    
def perform_inference(depth_arr):
    model_dir = '/home/developr/Documents/dexnet_baxter/models/GQCNN-4.0-PJ'
    model_name = 'GQCNN-4.0-PJ'
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
    csv_array = []
    for i in range(len(grasp_candidates)):
        # Position: [x y z] # Orientation: [qw qx qy qz]
        quaternion = grasp_candidates[i].pose().quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)
        csv_array.append([
            q_values[i],
            grasp_candidates[i].pose().position[0],
            grasp_candidates[i].pose().position[1],
            grasp_candidates[i].pose().position[2],
            euler[0],
            euler[1],
            euler[2],
            grasp_candidates[i].angle,
        ])
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
                grasp_candidates[i].endpoints, #tuple
                grasp_candidates[i].feature_vec
        ])
        )
    csv_array = np.array(csv_array)
    np.savetxt("dexnet_"+str(rospy.get_time())+".csv", csv_array, delimiter=",",fmt='%1.2f')
    #rospy.loginfo(candidate_array)
    if len(candidate_array)==0:
        return None
    if len(candidate_array)==0:
        return None
    best_idx = policy.select(grasp_candidates, q_values )
    result = np.array(candidate_array)
    print('Best idx: ', best_idx)
    return result, best_idx


def fill_goal_message_array(frame_id, position, orientation):
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = position[0]
    goal.pose.position.y = position[1]
    goal.pose.position.z = position[2]
    goal.pose.orientation.x = orientation[0]
    goal.pose.orientation.y = orientation[1]
    goal.pose.orientation.z = orientation[2]
    goal.pose.orientation.w = orientation[3]
    return goal


def fill_goal_message(frame_id, position, orientation):
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = rospy.Time.now()
    goal.pose.position = position
    goal.pose.orientation = orientation
    return goal


def pre_g_pose(target_pose):
    #
    orientation = None
    #orientation = [1, 0, 0, 0]  # face down
    z_offset = None
    z_offset = 0.15
    goal = target_pose
    if z_offset is not None:
        goal.pose.position.z = goal.pose.position.z + z_offset
    if orientation is not None:
        goal.pose.orientation.x = orientation[0]
        goal.pose.orientation.y = orientation[1]
        goal.pose.orientation.z = orientation[2]
        goal.pose.orientation.w = orientation[3]
    goal.pose = rotate_pose_msg_by_euler_angles(goal.pose, 0, -math.pi/2, 0)
    return goal

    
def main():
    rospy.init_node('export_grasp_pose', anonymous=True)
    #topic_name = '/kinect/depth/image_raw'
    topic_name = '/d435/depth/image_raw'

    camera_frame = 'd435_depth_optical_frame'
    world_frame = 'world'

    topic_name = rospy.get_param('~image_topic', topic_name)
    rospy.loginfo('wait for topic:')
    rospy.loginfo(topic_name)
    rospy.wait_for_message(topic_name, Image)
    rospy.loginfo('Received an image')
    listener = tf.TransformListener()
    listener.waitForTransform(world_frame, camera_frame, rospy.Time.now(), rospy.Duration(10.0))
    rospy.loginfo('Received a Transform')

    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("both_arms")

    left_current_pose = group.get_current_pose(end_effector_link='left_gripper')
    right_current_pose = group.get_current_pose(end_effector_link='right_gripper')
    print('left_current_pose', left_current_pose)
    print('right_current_pose', right_current_pose)
    pre_g = None
    gr = MoveGroupInterface("right_arm", "base")
    gl = MoveGroupInterface("left_arm", "base")
    msg = rospy.wait_for_message(topic_name, Image)

    arr = process_image_to_array(msg)
    rospy.loginfo(arr.shape)
    poses_candidates, best_idx = perform_inference(arr)

    if poses_candidates is None:
        rospy.logwarn("No candidates found")
    else:

        attempt_counter = best_idx
        while True:

            current_candidate = poses_candidates[attempt_counter]
            rospy.loginfo("Current attempt #: " + str(attempt_counter))
            print("Q", current_candidate[2])
            if current_candidate[2] < 0.05:
                attempt_counter = attempt_counter + 1
                continue
            #print(current_candidate)
            pose_camera_frame = fill_goal_message_array(camera_frame, current_candidate[0], current_candidate[1])
            pose_world_frame = listener.transformPose('world', pose_camera_frame)
            pre_g = pre_g_pose(pose_world_frame)

            result = gl.moveToPose(pre_g, "left_gripper", plan_only=False, planning_time=0.5, wait=True, max_velocity_scaling_factor=0.1)
            #print(result)
            #result = gr.moveToPose(pre_g_pose(pose_world_frame), "right_gripper", planning_time=0.1)
            if result.error_code.val == 1:
                pre_g = pose_world_frame
                print('arrived')
                break
            elif result.error_code.val == -4:
                #execution failed
                print('execution failed')
                break

            else:
                print(result.error_code.val)
                attempt_counter = attempt_counter + 1
                print('next')
                continue


if __name__ == '__main__':
    main()
