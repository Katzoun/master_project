# examples/python/visualization/non_blocking_visualization.py
import time
import open3d as o3d
import numpy as np


def prepare_data():
    pcd_data = o3d.data.DemoICPPointClouds()
    source_raw = o3d.io.read_point_cloud(pcd_data.paths[0])
    target_raw = o3d.io.read_point_cloud(pcd_data.paths[1])
    source = source_raw.voxel_down_sample(voxel_size=0.02)
    target = target_raw.voxel_down_sample(voxel_size=0.02)

    trans = [[0.862, 0.011, -0.507, 0.0], [-0.139, 0.967, -0.215, 0.7],
             [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]]
    source.transform(trans)
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    source.transform(flip_transform)
    target.transform(flip_transform)
    return source, target


def demo_combined_visualization(mesh):
    """Kombinovaný visualizer s editing a key callbacks"""
    # Fallback: Pouze editing visualizer
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Editing Visualizer")
    vis.add_geometry(mesh)
    opt = vis.get_render_option()
    opt.mesh_show_back_face = True
    print("Editing visualizer spuštěn. Použijte myš pro editaci.")
    vis.run()
    vis.destroy_window()

    idxs = vis.get_picked_points()   # seznam indexů vrcholů (ints)
    verts = np.asarray(mesh.vertices)
    for i in idxs:
        print("picked vertex", i, "->", verts[i])


def visualization(mesh):

    vis = o3d.visualization.Visualizer()
    opt = vis.get_render_option()   
    print(opt)
    vis.create_window()
    vis.add_geometry(mesh)
    # vis.run()
    # vis.destroy_window()
    i = 0
    while i<200:
        vis.poll_events()
        # print(vis.get_view_control().convert_to_pinhole_camera_parameters())
        vis.update_renderer()
        # PickedPoints = vis.get_picked_points()
        # print(PickedPoints)
        i += 1
        time.sleep(0.1)

    vis.destroy_window()



    # for i in range(icp_iteration):
    #     reg_p2l = o3d.pipelines.registration.registration_icp(
    #         source, target, threshold, np.identity(4),
    #         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    #         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
    #     source.transform(reg_p2l.transformation)
    #     vis.update_geometry(source)
    #     vis.poll_events()
    #     vis.update_renderer()
    #     if save_image:
    #         vis.capture_screen_image("temp_%04d.jpg" % i)
    # vis.destroy_window()

    # o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)


if __name__ == '__main__':
    # Load mesh from ply file
    # Set o3d verbosity to high
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    mesh = o3d.io.read_triangle_mesh("frames/marching_mesh.ply")
    # Check and compute vertex normals if not present
    print(f"  Has vertex normals: {mesh.has_vertex_normals()}")
    if not mesh.has_vertex_normals():
        print("  Computing vertex normals...")
        mesh.compute_vertex_normals()
        
    # Spustíme kombinovaný visualizer
    demo_combined_visualization(mesh)