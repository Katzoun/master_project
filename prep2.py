
SOURCE_PATH = "abb_ws_new/saved_frames/source_outlier_radius.ply"   # zdroj (přetransformuje se)
TARGET_PATH = "abb_ws_new/saved_frames/target_outlier_radius.ply"   # cíl (referenční rámec)


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


    # print("Initial alignment")
    # evaluation = o3d.pipelines.registration.evaluate_registration(
    #     source, target, threshold, trans_init)
    # print(evaluation, "\n")

    # point_to_plane_icp(source, target, threshold, trans_init)
    point_to_plane_icp_normal(source, target, threshold, trans_init)
    target_downsample_highly = target_downsample.voxel_down_sample(voxel_size=0.5)
