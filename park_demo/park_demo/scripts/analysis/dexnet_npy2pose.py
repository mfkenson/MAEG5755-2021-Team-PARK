#!/usr/bin/env python3.6
import numpy as np
import pandas as pd


from autolab_core import YamlConfig, Logger
from perception import (BinaryImage, CameraIntrinsics, ColorImage, DepthImage,
                        RgbdImage)
from visualization import Visualizer2D as vis

from gqcnn.grasping import (RobustGraspingPolicy,
                            CrossEntropyRobustGraspingPolicy, RgbdImageState,
                            FullyConvolutionalGraspingPolicyParallelJaw,
                            FullyConvolutionalGraspingPolicySuction)
from gqcnn.utils import GripperMode

from math import pi

    
def perform_inference(depth_arr):
    model_dir = '/home/developr/Documents/dexnet_baxter/models/GQCNN-4.0-PJ'
    model_params = '/home/developr/Documents/dexnet_baxter/dexnet_deps/gqcnn/cfg/examples/replication/dex-net_4.0_pj.yaml'
    camera_intr_filename = '/home/developr/5755_ws/src/team-park/park_dexnet/config/camera_d435_gazebo.intr'
    camera_intr = CameraIntrinsics.load(camera_intr_filename)
    config = YamlConfig(model_params)
    policy_config = config["policy"]
    policy_config["metric"]["gqcnn_model"] = model_dir
    depth_im = DepthImage(depth_arr, frame=camera_intr.frame)
    color_im = ColorImage(np.zeros([depth_im.height, depth_im.width,
                                        3]).astype(np.uint8),
                            frame=camera_intr.frame)
    segmask = BinaryImage(255 *
                          np.ones(depth_im.shape).astype(np.uint8),
                          frame=color_im.frame)
    rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)
    state = RgbdImageState(rgbd_im, camera_intr, segmask=segmask)
    policy = CrossEntropyRobustGraspingPolicy(policy_config)
    grasp_candidates, q_values = policy.action_set(state)
    candidate_array = []
    for i in range(len(grasp_candidates)):
        # Position: [x y z] # Orientation: [qw qx qy qz]
        candidate_array.append(np.array([
            #grasp_candidates[i].pose().to_frame,
            np.round(grasp_candidates[i].pose().position, 6),
            np.round(grasp_candidates[i].pose().quaternion,6),
            q_values[i],
            grasp_candidates[i].center.vector,  # center is a type of autolab_core.Point
            grasp_candidates[i].angle,
            grasp_candidates[i].depth,
            grasp_candidates[i].width,
            grasp_candidates[i].contact_points,
            grasp_candidates[i].contact_normals,
            grasp_candidates[i].axis,
            grasp_candidates[i].approach_angle,
            grasp_candidates[i].width_px,
            grasp_candidates[i].endpoints,
            grasp_candidates[i].feature_vec,
            grasp_candidates[i].width_px,
        ])
            )
    if len(candidate_array)==0:
        return None
    best_idx = policy.select(grasp_candidates, q_values )
    result = np.array(candidate_array)
    print('Best idx: ', best_idx)
    return result, best_idx
    #df = pd.DataFrame(result, columns =['position', 'quaternion', 'q_value'])
    #sorted_arr = df.sort_values('q_value', ascending=False).to_numpy()
    #return sorted_arr

    
def main():
    npy_file_path = '_d435_depth_image_raw.npy'
    arr = np.load(npy_file_path)
    arr = arr/1000.0
    poses_candidates, best_idx = perform_inference(arr)
    if poses_candidates is None:
        print("No candidates found")
    else:
        print("Best candidate")
        print("position", poses_candidates[best_idx][0])
        print("quaternion", poses_candidates[best_idx][1])
        print("q value", poses_candidates[best_idx][2])
    np.save('_d435_dexnet_pose_'+ str(best_idx)+'.npy', poses_candidates)
if __name__ == '__main__':
    main()
