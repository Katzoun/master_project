import ast
import numpy as np
from scipy.spatial.transform import Rotation as R

def string_to_list(input_string):
    """
    Converts a string representation to a List.
    example format '[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]]'.
    """
    # Replace ABB-style booleans with Python booleans
    input_string = input_string.replace('TRUE', 'True').replace('FALSE', 'False')
    #print(string)
    try:
        return ast.literal_eval(input_string)
    except Exception as e:
        #print(f"Error: {e}")
        raise ValueError(f"Invalid string: {input_string}") from e
    
def list_to_string(input_list):
    """
    Converts a List representation to a string.
    The List should be in the format [True, [[0, 0, 0], [1, 0, 0, 0]], [0.001, [0, 0, 0.001], [1, 0, 0, 0], 0, 0, 0]].
    """
    try:
        # Convert the list to a string representation
        str_input = str(input_list)
        # Replace Python booleans with ABB-style booleans
        return str_input.replace('True', 'TRUE').replace('False', 'FALSE')
    except Exception as e:
        #print(f"Error: {e}")
        raise ValueError(f"Invalid list input: {input_list}") from e

def pose_vector_to_tf_matrix(params: np.ndarray) -> np.ndarray:
    """
    Creates a 4x4 transformation matrix from a vector of position and rotation.

    This function takes a 7D vector representing position and rotation (qaternion),
    and constructs a 4x4 homogeneous transformation matrix.

    Parameters:
        params (np.ndarray): A 7D vector in the form [tx, ty, tz, qx, qy, qz, qw]
            where [tx, ty, tz] is the translation (in any unit)
            and [qx, qy, qz, qw] is the rotation (as a quaternion)

    Returns:
        np.ndarray: A 4x4 homogeneous transformation matrix

    Notes:
        - The function uses scipy.spatial.transform.Rotation for rotation calculations
        - The rotation is applied using the xyz convention (rotations about fixed axes)
        - The returned matrix is in the form:
        [R R R tx]
        [R R R ty]
        [R R R tz]
        [0 0 0  1]
        where R represents the 3x3 rotation matrix and [tx, ty, tz] is the translation
    """
    # Split the vector into translation and rotation
    tx, ty, tz = params[:3]
    q1, q2, q3, q4 = params[3:]

    rotation_matrix = R.from_quat([q1, q2, q3, q4], scalar_first=True).as_matrix()

    # Create 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix  # Place rotation matrix
    transformation_matrix[:3, 3] = [tx, ty, tz]  # Place translation vector
    
    return transformation_matrix

def tf_matrix_to_pose_vector(transformation_matrix: np.ndarray, use_euler: bool = False) -> np.ndarray:
    """
    Converts a 4x4 transformation matrix into a 6D vector representing position and rotation.

    This function takes a 4x4 homogeneous transformation matrix and extracts
    the translation and rotation information to construct a 6D vector.

    Parameters:
        transformation_matrix (np.ndarray): A 4x4 homogeneous transformation matrix
        use_euler (bool): If True, returns Euler angles instead of quaternion vector.

    Returns:
        np.ndarray: A 6D vector in the form [tx, ty, tz, rx, ry, rz]
            where [tx, ty, tz] is the translation (in any unit)
            and [rx, ry, rz] is the rotation (in radians)

    Notes:
        - The rotation is computed using the rotation matrix extracted from the transformation matrix.
        - The rotation vector is derived using the rotation matrix to represent rotations about the fixed axes.
    """
    # Extract translation vector from the transformation matrix
    tx, ty, tz = transformation_matrix[:3, 3]

    # Extract rotation matrix
    rotation_matrix = transformation_matrix[:3, :3]
    
    # Calculate rotation vector from the rotation matrix
    rotation = R.from_matrix(rotation_matrix)
    if use_euler:
        rx, ry, rz = rotation.as_euler('xyz', degrees=True)
        res = np.array([tx, ty, tz, rx, ry, rz])
    else:
        #quaternion
        q1, q2, q3, q4 = rotation.as_quat(scalar_first=True)
        res = np.array([tx, ty, tz, q1, q2, q3, q4])

    # Construct and return the
    return res

def abb_quaternion_to_euler_xyz(q1, q2, q3, q4):
    """
    Convert ABB quaternion to Euler angles in ABB convention (matching RobotStudio).
    
    ABB uses xyz intrinsic rotations.
    
    Parameters:
        q1, q2, q3, q4: ABB quaternion components (q1=x, q2=y, q3=z, q4=w)
    
    Returns:
        np.ndarray: Euler angles [rx, ry, rz] in degrees (ABB XYZ convention)
    """
    # ABB quaternion format: q1=x, q2=y, q3=z, q4=w
    # Convert to scipy format [x, y, z, w] and use xyz intrinsic rotations
    rotation = R.from_quat([q1, q2, q3, q4], scalar_first=True)

    # ABB uses xyz intrinsic rotations
    euler_xyz = rotation.as_euler('xyz', degrees=True)
    
    return euler_xyz
