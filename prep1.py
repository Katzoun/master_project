
SOURCE_PATH = "abb_ws_new/saved_frames/scan0.ply"   # zdroj (přetransformuje se)
TARGET_PATH = "abb_ws_new/saved_frames/scan1.ply"   # cíl (referenční rámec)


# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# Copyright (c) 2018-2024 www.open3d.org
# SPDX-License-Identifier: MIT
# ----------------------------------------------------------------------------
"""ICP (Iterative Closest Point) registration algorithm"""

import open3d as o3d
import numpy as np
import copy

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw([source_temp, target_temp])


# def point_to_point_icp(source, target, threshold, trans_init):
#     print("Apply point-to-point ICP")
#     reg_p2p = o3d.pipelines.registration.registration_icp(
#         source, target, threshold, trans_init,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint())
#     print(reg_p2p)
#     print("Transformation is:")
#     print(reg_p2p.transformation, "\n")
#     draw_registration_result(source, target, reg_p2p.transformation)
def point_to_plane_icp_normal(source, target, threshold, trans_init):
    print("Apply point-to-plane ICP")

    p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    reg_p2l = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init, p2l)

    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation, "\n")
    draw_registration_result(source, target, reg_p2l.transformation)


def point_to_plane_icp(source, target, threshold, trans_init):
    print("Apply point-to-plane ICP")

    loss = o3d.pipelines.registration.TukeyLoss(k=0.1)
    p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
    reg_p2l = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init, p2l)

    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation, "\n")
    draw_registration_result(source, target, reg_p2l.transformation)


if __name__ == "__main__":
    # pcd_data = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud(TARGET_PATH)
    target = o3d.io.read_point_cloud(SOURCE_PATH)
    # visualize the initial alignment

    threshold = 0.5
    trans_init = np.eye(4)
    #downsample
    source_downsample = source.voxel_down_sample(voxel_size=0.02)
    target_downsample = target.voxel_down_sample(voxel_size=0.02)
    source_downsample_highly = source_downsample.voxel_down_sample(voxel_size=0.5)
    target_downsample_highly = target_downsample.voxel_down_sample(voxel_size=0.5)
    print("Source downsampled points:", np.asarray(source_downsample.points).shape[0])
    print("Target downsampled points:", np.asarray(target_downsample.points).shape[0])
    print("Source downsampled highly points:", np.asarray(source_downsample_highly.points).shape[0])
    print("Target downsampled highly points:", np.asarray(target_downsample_highly.points).shape[0])



    # source_outlier, ind = source_downsample_highly.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    # print("Source points after outlier removal:", np.asarray(source_outlier.points).shape[0])
    # # draw_registration_result(source, target, trans_init)
    # #print no of points
    # display_inlier_outlier(source_downsample_highly, ind)

    print("Radius outlier removal")
    source_outlier_radius, ind_radius = source_downsample_highly.remove_radius_outlier(nb_points=100, radius=10)
    # display_inlier_outlier(source_downsample_highly, ind_radius)
    print("Source points after outlier removal:", np.asarray(source_outlier_radius.points).shape[0])

    print("Radius outlier removal")
    target_outlier_radius, ind_radius = target_downsample_highly.remove_radius_outlier(nb_points=100, radius=10)
    # display_inlier_outlier(target_downsample_highly, ind_radius)
    print("Target points after outlier removal:", np.asarray(target_outlier_radius.points).shape[0])

    #save these
    ret1 = o3d.io.write_point_cloud("abb_ws_new/saved_frames/source_outlier_radius.ply", source_outlier_radius)
    ret2 = o3d.io.write_point_cloud("abb_ws_new/saved_frames/target_outlier_radius.ply", target_outlier_radius)
    print("Saved source_outlier_radius.ply = ", ret1)
    print("Saved target_outlier_radius.ply = ", ret2)

    # print("Initial alignment")
    # evaluation = o3d.pipelines.registration.evaluate_registration(
    #     source, target, threshold, trans_init)
    # print(evaluation, "\n")

    # point_to_point_icp(source, target, threshold, trans_init)
    # point_to_plane_icp(source, target, threshold, trans_init)
    # point_to_plane_icp_normal(source, target, threshold, trans_init)
