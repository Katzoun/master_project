
import numpy as np
import os
import pandas as pd



if __name__ == "__main__":
    directory = "calib2"
    #open file calib_poses and camera points
    with open(os.path.join(directory, "calib_poses2.txt"), "r") as f:
        calib_poses = np.loadtxt(f)

    with open(os.path.join(directory, "camera_points.txt"), "r") as f:
        camera_points = np.loadtxt(f)

    #remove first 7 columns of camera_points
    camera_points = camera_points[:, 7:]

    #combine both arrays
    combined = np.hstack((calib_poses, camera_points))

    #save combined array to new file
    np.savetxt(os.path.join(directory, "combined11.txt"), combined)


