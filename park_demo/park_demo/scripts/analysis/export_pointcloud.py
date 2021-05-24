#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import ros_numpy
import random
import matplotlib
from matplotlib import pyplot as plt
from sklearn import linear_model
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
np.set_printoptions(suppress=True, precision=4)
seed=1000
random.seed(seed)
np.random.seed(seed)


def export_pointcloud(ros_point_cloud, file_name):
    rospy.loginfo('Got a message')
    pc = ros_numpy.numpify(ros_point_cloud)
    rospy.loginfo('frame: ' + ros_point_cloud.header.frame_id)
    rospy.loginfo(pc.shape)
    np.save(file_name, pc)
    rospy.loginfo("done")
    process_and_plot(pc)


def process_and_plot(pc):
    pc_shape = pc.shape
    points = np.zeros((3, pc.shape[0]))
    points[0] = pc['x'].flatten()
    points[1] = pc['y'].flatten()
    points[2] = pc['z'].flatten()
    npts = len(points[0])
    max_dist = 0.8
    too_large_z_idx = np.argwhere(points[2] >  max_dist)#filter remove those larger than certain distance
    points_filtered = np.zeros((3, npts - len(too_large_z_idx)))

    points_filtered[0] = np.delete(points[0], too_large_z_idx)
    points_filtered[1] = np.delete(points[1], too_large_z_idx)
    points_filtered[2] = np.delete(points[2], too_large_z_idx)
    npts = npts - len(too_large_z_idx)


    xyz = (points_filtered.T)
    xy = xyz[:, :2]
    z = xyz[:, 2]
    pts = points_filtered

    ransac = linear_model.RANSACRegressor(residual_threshold=None, max_trials=100, random_state=seed)
    ransac.fit(xy, z)
    a, b = ransac.estimator_.coef_  # coefficients
    d = ransac.estimator_.intercept_  # intercept
    inlier_mask = ransac.inlier_mask_
    # Z = aX + bY + d
    # aX + by - Z = -d
    ransac_vec = np.array([a, b, -1]) / -d
    estimated_dist_ransec = 1 / np.linalg.norm(ransac_vec)
    print('[ransac] estimated dist', estimated_dist_ransec)
    print('[ransac] estimated normal vector:', ransac_vec)


    min_x, max_x, min_y, max_y = np.min(pts[0]) - 1, np.max(pts[0]) + 2, np.min(pts[1]) - 1, np.max(pts[1]) + 2
    step = 1.5
    X, Y = np.meshgrid(np.arange(min_x, max_x, step=step), np.arange(min_y, max_y, step=step))
    Z_ransac = np.zeros(X.shape)
    for r in range(X.shape[0]):
        for c in range(X.shape[1]):
            Z_ransac[r, c] = (1 - (ransac_vec[0] * X[r, c] + ransac_vec[1] * Y[r, c])) / ransac_vec[2]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    point_scale = 50000 / npts  # point size scale
    downsample = 10
    ax.scatter(pts[0][::downsample], pts[1][::downsample], pts[2][::downsample], color='red', s=point_scale)
    ax.plot_wireframe(X, Y, Z_ransac, color='black')  # ransac

    plt.show()

def main():
    rospy.init_node('export_pointcloud', anonymous=True)
    topic_name = '/d435/depth/color/points'
    topic_name = rospy.get_param('~image_topic', topic_name)
    rospy.loginfo('wait for topic:')
    rospy.loginfo(topic_name)
    file_name = topic_name.replace('/', '_') + '.npy'
    msg = rospy.wait_for_message(topic_name, PointCloud2)
    export_pointcloud(msg, file_name)


if __name__ == '__main__':
    main()
