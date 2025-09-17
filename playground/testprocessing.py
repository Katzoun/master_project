# Image Processing
# import point cloud captured from photoneo camera in point cloud .ply
import sys
import os
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import struct

print(os.getcwd())

def extract_photoneo_camera_info(filepath):
    """Extract camera information from Photoneo PLY file"""
    
    camera_info = {}
    
    with open(filepath, 'rb') as f:
        # Skip to end of header
        while True:
            line = f.readline().decode('ascii').strip()
            if line == 'end_header':
                break
        
        # Read vertex data (skip for now - we want camera data)
        # Each vertex has: x,y,z,nx,ny,nz,r,g,b,texture32,depth32 (44 bytes total)
        vertex_count = 3186816 # From header
        vertex_size = 35  
        f.seek(vertex_count * vertex_size, 1)  # Skip vertex data
        
        # Read camera element (1 camera)
        camera_data = f.read(12 * 4)  # 12 float properties
        camera_values = struct.unpack('<12f', camera_data)
        
        camera_info['camera'] = {
            'view_position': [camera_values[0], camera_values[1], camera_values[2]],  # view_px, view_py, view_pz
            'x_axis': [camera_values[3], camera_values[4], camera_values[5]],         # x_axisx, x_axisy, x_axisz
            'y_axis': [camera_values[6], camera_values[7], camera_values[8]],         # y_axisx, y_axisy, y_axisz
            'z_axis': [camera_values[9], camera_values[10], camera_values[11]]        # z_axisx, z_axisy, z_axisz
        }
        
        # Read phoxi_frame_params (1 element)
        frame_data = f.read(8 * 4)  # 6 uint32 + 3 float + 1 int32 = 8*4 bytes
        frame_values = struct.unpack('<IIIFFFFI', frame_data)
        
        camera_info['frame_params'] = {
            'width': frame_values[0],
            'height': frame_values[1], 
            'frame_index': frame_values[2],
            'start_time': frame_values[3],
            'duration': frame_values[4],
            'computation_duration': frame_values[5],
            'transfer_duration': frame_values[6],
            'total_scan_count': frame_values[7]
        }
        
        # Read camera matrix (3x3 matrix)
        matrix_data = f.read(9 * 4)  # 9 floats
        matrix_values = struct.unpack('<9f', matrix_data)
        
        camera_info['camera_matrix'] = np.array(matrix_values).reshape(3, 3)
        
        # Read distortion matrix
        distortion_data = f.read(14 * 4)  # 14 floats
        distortion_values = struct.unpack('<14f', distortion_data)
        
        camera_info['distortion_matrix'] = np.array(distortion_values)
        
        # Read camera resolution
        resolution_data = f.read(2 * 4)  # 2 floats
        resolution_values = struct.unpack('<2f', resolution_data)
        
        camera_info['camera_resolution'] = {
            'width': resolution_values[0],
            'height': resolution_values[1]
        }
        
        # Read frame binning
        binning_data = f.read(2 * 4)  # 2 floats
        binning_values = struct.unpack('<2f', binning_data)
        
        camera_info['frame_binning'] = {
            'horizontal': binning_values[0],
            'vertical': binning_values[1]
        }
    
    return camera_info


if __name__ == "__main__":

    frames_dir = "/home/robolab2/master_project/frames"

    # List all .ply files in the directory
    ply_files = [f for f in os.listdir(frames_dir) if f.endswith('.ply')]
    ply_files.sort()  # Sort files to ensure correct order
    print(f"Found {len(ply_files)} .ply files.")

    #open scan_28.ply a visualize 2d image of point cloud from camera perspective
    ply_name = "scan_28.ply"
    ply_path = os.path.join(frames_dir, ply_name)
    camera_info = extract_photoneo_camera_info(ply_path)

    # Print extracted information
    print("=== CAMERA INFORMATION ===")
    print(f"Camera position: {camera_info['camera']['view_position']}")
    print(f"Camera X-axis: {camera_info['camera']['x_axis']}")
    print(f"Camera Y-axis: {camera_info['camera']['y_axis']}")
    print(f"Camera Z-axis: {camera_info['camera']['z_axis']}")

    print(f"\nFrame dimensions: {camera_info['frame_params']['width']} x {camera_info['frame_params']['height']}")
    print(f"Frame index: {camera_info['frame_params']['frame_index']}")

    print(f"\nCamera matrix (3x3):")
    print(camera_info['camera_matrix'])

    print(f"\nCamera resolution: {camera_info['camera_resolution']['width']} x {camera_info['camera_resolution']['height']}")


