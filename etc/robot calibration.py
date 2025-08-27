import struct
import time
from utils.harvester_singleton import get_harvester, get_image_acquisition
from  utils.rwsprovider import RWSClient
from utils.rwswrappers import RWSWrappers
import utils.utilities as utilities
import numpy as np
import os


robotIP = "192.168.0.37"
username = "Admin"
password = "robotics"
port = 443 # real robot 443
directory = "calib2"

if __name__ == "__main__":

    try:
        print("Current Working Directory:", os.getcwd())    
        os.makedirs(directory, exist_ok=True)
        iter = 0
        rwsCl = RWSClient(robotIP, username, password,port)
        rwsWrap  = RWSWrappers(rwsCl)
        # harvester = get_harvester()
        # ia = get_image_acquisition("PhotoneoTL_DEV_2019-06-011-LC3")
        # features = ia.remote_device.node_map
        rwsCl.login()


        print("Logged in:", rwsCl.is_logged_in())
        # Example of getting the clock
        clock = rwsWrap.get_clock()
        #print("Clock:", clock)
        #load calibration matrix from calib_mat.txt
        # calib_mat = np.loadtxt("src/ros2_rws/calib_mat.txt")
        # print(type(calib_mat))
        # print(calib_mat.shape)
        # print("Calibration Matrix Loaded:\n", calib_mat)
        # calib_pose = rwsutils.tf_matrix_to_pose_vector(calib_mat, use_euler=True)
        # print("Calibration Pose 6D Vector:\n", calib_pose)

        print("Mastership edit:", rwsWrap.is_master("edit"))

        # #cyclically wait for user keypress
        # calib_mat = np.loadtxt("src/ros2_rws/calib_mat.txt")
        # print(type(calib_mat))
        # print(calib_mat.shape)
        input("Press enter to scan first pose")


        while True:
            #get robot position
            pose = rwsWrap.get_robot_cartesian()[0]
            print("Robot Cartesian Pose:", pose)
            
            #open or create new file and append last pose to file
            with open(os.path.join(directory, "calib_poses1.txt"), "a") as f:
                f.write(" ".join(f"{x:.6f}" for x in pose) + "\n")
    
            iter += 1
            print(f"Iteration: {iter}")

            input("Press Enter to continue...")
            print("Continuing...")



            # # print(f"[{iter}] Robot Cartesian Pose:", pose)
            # tf_mat = rwsutils.pose_vector_to_tf_matrix(pose)
            # # print(f"[{iter}] Robot TCP TF:\n", tf_mat)
            # camera_base_tf = np.matmul(tf_mat, calib_mat)
            # print(f"[{iter}] Camera to Base TF:\n", camera_base_tf)
            
            # pose_6d = rwsutils.tf_matrix_to_pose_vector(camera_base_tf, use_euler=True)
            # print(f"[{iter}] Camera to Base Pose 6D Vector:\n", pose_6d)
            # camera_rot_mat = camera_base_tf[:3, :3]
            # camera_trans_vect = camera_base_tf[:3, 3].tolist()

            # camera_rot_mat_reshape = camera_rot_mat.reshape(1, 9).tolist()

            # coordinate_space = features.CoordinateSpace.value
            # features.CoordinateSpace.value = 'RobotSpace'
            # # `Custom` or `Robot`
            # transformation_space_selector = features.TransformationSpaceSelector.value
            # features.TransformationSpaceSelector.value = 'Robot'

    
        #     robot_transformation_rotation_matrix_new_values = camera_rot_mat_reshape[0]
        #     robot_transformation_rotation_matrix_length = features.RobotTransformationRotationMatrix.length
        #     robot_transformation_rotation_matrix_bytes = features.RobotTransformationRotationMatrix.get(robot_transformation_rotation_matrix_length)
        #     robot_transformation_rotation_matrix = struct.unpack('9d', robot_transformation_rotation_matrix_bytes)
        #     robot_transformation_rotation_matrix_new_bytes = struct.pack('9d', *robot_transformation_rotation_matrix_new_values)

        #     robot_transformation_translation_vector_new_values = camera_trans_vect
        #     robot_transformation_translation_vector_length = features.RobotTransformationTranslationVector.length
        #     robot_transformation_translation_vector_bytes = features.RobotTransformationTranslationVector.get(robot_transformation_translation_vector_length)
        #     robot_transformation_translation_vector = struct.unpack('3d', robot_transformation_translation_vector_bytes)
        #     robot_transformation_translation_vector_new_bytes = struct.pack('3d', *robot_transformation_translation_vector_new_values)


        #     features.RobotTransformationTranslationVector.set(robot_transformation_translation_vector_new_bytes)
        #     features.RobotTransformationRotationMatrix.set(robot_transformation_rotation_matrix_new_bytes)


        #     ia.start()
        #     # Trigger frame by calling property's setter.
        #     # Must call TriggerFrame before every fetch.
        #     features.TriggerFrame.execute() # trigger first frame
        #     with ia.fetch(timeout=10.0) as buffer:
        #         # grab first frame
        #         # do something with first frame
        #         print(buffer)

        #         save_last_scan_path = os.path.join("/home/robolab2/abb_ws_new/saved_frames", f"scan{iter}.ply")
        #         print("--> save_last_scan_path: ", save_last_scan_path)
        #         features.SaveLastScanFilePath.value = save_last_scan_path
        #         features.SaveLastScanFrameId.value= -1

                
        #         json_options = '{"UseCompression": true}'
        #         features.SaveLastScanJsonOptions.value = json_options

        #         features.SaveLastScan.execute()

        #     iter += 1
        #     input("Press Enter to continue...")
        #     print("Continuing...")
        #     time.sleep(1)

        # pose_euler = rwsWrap.get_robot_cartesian_euler()[0]
        # print("Robot Cartesian Pose euler:", pose_euler)  

        # print("Mastership mot:", rwsWrap.is_master("motion"))
        # print(rwsWrap.release_mastership("edit"))
        #print(rwsWrap.create_dipc_queue("T_ROB1"))
        # print(rwsWrap.get_dipc_queues())
        # print(rwsWrap.get_dipc_queue_info())
        # print(rwsWrap.send_dipc_message('num;22'))

        # print(rwsWrap.set_rapid_symbol_string("test_routine_buffer", "routine_name_input", "TRobRAPID"))
        # print(rwsWrap.set_rapid_symbol_int(2, "current_state", "TRobMain"))

        # print(rwsWrap.send_dipc_message('string;"cccc"'))
        # print(rwsWrap.send_dipc_message('string;"ahoj"'))
        # print(rwsWrap.send_dipc_message('string;"runMoveJ"'))

        # print(rwsWrap.send_dipc_message('robtarget;[[1000,0,700],[0,0,-1.0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]'))
        # print(rwsWrap.send_dipc_message('robtarget;[[1200,0,700],[0,0,-1.0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]'))
        # print(rwsWrap.send_dipc_message('robtarget;[[800,0,700],[0,0,-1.0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]'))
        # print(rwsWrap.send_dipc_message('robtarget;[[1000,500,700],[0,0,-1.0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]'))
        # print(rwsWrap.send_dipc_message('robtarget;[[-600,0,800],[0,0,-1.0,0],[0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]]'))

        # print(rwsWrap.read_dipc_message(timeout=5))
        # print(rwsWrap.read_dipc_message(timeout=5))
        #(rwsWrap.send_dipc_message("Hello world!"))
        
        # print(rwsWrap.start_rapid_script())

        # robt = rwsWrap.get_rapid_robtarget("move_robtarget_input", "TRobRAPID")[0]

        # robt[0][0] = 800

        # print(rwsWrap.set_rapid_symbol_list(robt, "move_robtarget_input", "TRobRAPID"))


     




        time.sleep(1)


    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        rwsCl.logout()
        ia.destroy()