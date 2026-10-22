import numpy as np
import gtsam

def getBearing3D(pose1, pose2, std=None, add_noise=False):

    # Translation in global frame
    posdiff = pose1.translation() - pose2.translation()


    # Bearing info (received)
    phi = np.arctan2(posdiff[1], posdiff[0])    # azimuth
    if phi < 0:
        phi += 2*np.pi

    r = np.linalg.norm(posdiff)
    theta = np.arccos(posdiff[2]/r)  # elevation SIN OR COS?
    bearing = np.array([phi, theta])

    return bearing
    # pos1 = pos1.translation()
    # pos2 = pos2.translation()

    # posdiff = pos1 - pos2

    # # Calculate azimuth between both agents
    # # print("pos1", pos1)
    # # print("pos2", pos2)
    # phi = np.arctan2(posdiff[1], posdiff[0])
    # if phi < 0:
    #     phi = phi + 2*np.pi

    # # Calculate elevation between both agents TODO make sure in correct quadrant
    # theta = np.arctan2(posdiff[2], posdiff[0])

    # # r = np.linalg.norm(pos1 - pos2)
    # # r = np.sqrt(posdiff[0]**2 + posdiff[1]**2)
    # # theta = np.arcsin(posdiff[2]/r)
    # # theta = np.arctan2(posdiff[2], r)

    # bearing = [phi, theta]
    
def getHBetween(prevPose, currentPose, std):

    '''std is 6 long for 3d [Rx,Ry,Rz,Tx,Ty,Tz]'''

    pose2_wrt_pose1 = prevPose.between(currentPose)
    # pose2_wrt_pose1 = prevPose.inverse().compose(currentPose)
    noise = np.random.normal([0, 0, 0, 0, 0, 0], std)
    delta_noise = gtsam.Pose3(gtsam.Rot3.RzRyRx(noise[0:3]), gtsam.Point3(noise[3:]))

    # Noisy poses
    pose2_wrt_pose1_noisy = pose2_wrt_pose1.compose(delta_noise)  

    return pose2_wrt_pose1, pose2_wrt_pose1_noisy

def addVectorNoise(v, std=0.01):
    v = v + np.random.normal(0, std, np.shape(v))
    return v

def integrate_dvl(pose, velocity, dt):
    # Integrate velocity to update position
    pose[:3] += velocity[:3] * dt
    
    return pose

def getBetweenPose(pose1_wrt_world, pose2_wrt_world):

    '''take in gtsam object, return pose2_wrt_pose1'''

    # pose2_wrt_pose1 = pose2_wrt_world.compose(pose1_wrt_world.inverse())
    pose2_wrt_pose1 = pose1_wrt_world.inverse().compose(pose2_wrt_world)

    return pose2_wrt_pose1

def get_azimuth_elevation(matrix1, matrix2):
    # Extract translation vectors from the homogeneous matrices
    translation1 = matrix1[:3, 3]
    translation2 = matrix2[:3, 3]

    # Compute the vector pointing from the first camera pose to the second camera pose
    direction_vector = translation2 - translation1

    # Convert the direction vector to spherical coordinates (azimuth, elevation, and distance)
    r = np.linalg.norm(direction_vector)
    theta = np.arctan2(direction_vector[1], direction_vector[0])
    phi = np.arctan2(direction_vector[2], np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2))

    # Convert azimuth and elevation from radians to degrees
    azimuth_deg = np.degrees(theta)
    elevation_deg = np.degrees(phi)

    return azimuth_deg, elevation_deg, r

def compose(posei, posej, H1=None, H2=None):
    inverse_g = np.linalg.inv(posej)
    if H1 is not None:
        H1[...] = inverse_g.T  # AdjointMap of the inverse
    if H2 is not None:
        H2[...] = np.eye(len(posej))  # Identity matrix
    return posei@posej

def HfromRT2D(R, t):
    """
    Create a 3x3 homogeneous transformation matrix given a rotation matrix (R) and a translation vector (t).

    Args:
    - R (np.ndarray): 2x2 rotation matrix.
    - t (np.ndarray): Translation vector of shape (2,).

    Returns:
    - H (np.ndarray): 4x4 homogeneous transformation matrix.
    """
    # Create a 4x4 identity matrix
    H = np.eye(3)

    # Assign the rotation matrix to the top-left 3x3 block
    H[:2, :2] = R

    # Assign the translation vector to the rightmost column
    H[:2, 2] = t

    return H

def HfromRT(R, t):
    """
    Create a 4x4 homogeneous transformation matrix given a rotation matrix (R) and a translation vector (t).

    Args:
    - R (np.ndarray): 3x3 rotation matrix.
    - t (np.ndarray): Translation vector of shape (3,).

    Returns:
    - H (np.ndarray): 4x4 homogeneous transformation matrix.
    """
    # Create a 4x4 identity matrix
    H = np.eye(4)

    # Assign the rotation matrix to the top-left 3x3 block
    H[:3, :3] = R

    # Assign the translation vector to the rightmost column
    H[:3, 3] = t

    return H

def RTfromH(H):
    """
    Extract the rotation matrix (R) and translation vector (t) from a 4x4 homogeneous transformation matrix (H).

    Args:
    - H (np.ndarray): 4x4 homogeneous transformation matrix.

    Returns:
    - R (np.ndarray): 3x3 rotation matrix.
    - t (np.ndarray): Translation vector of shape (3,).
    """
    # Extract the rotation matrix (top-left 3x3 block)
    R = H[:3, :3]

    # Extract the translation vector (rightmost column)
    t = H[:3, 3]

    return R, t

def create_rotation_matrix_from_euler(euler_angles):
    """
    Create a 3x3 rotation matrix from Euler angles.

    Args:
    - euler_angles (np.ndarray): Euler angles [roll, pitch, yaw].

    Returns:
    - R (np.ndarray): 3x3 rotation matrix.
    """
    roll, pitch, yaw = euler_angles
    rotation_matrix = np.array([[np.cos(yaw) * np.cos(pitch),
                                np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll),
                                np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)],
                               [np.sin(yaw) * np.cos(pitch),
                                np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll),
                                np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)],
                               [-np.sin(pitch),
                                np.cos(pitch) * np.sin(roll),
                                np.cos(pitch) * np.cos(roll)]])

    return rotation_matrix

# TODO fix orientation
def integrate_imu(prev_pose, accel, ang_vel, dt):
    """
    Integrate IMU measurements to estimate pose using Euler integration. https://robotics.stackexchange.com/questions/16757/what-is-the-algorithm-to-get-position-linear-displacement-and-linear-velocity 

    Args:
    - accel (np.ndarray): Acceleration measurements [accel_x, accel_y, accel_z].
    - ang_vel (np.ndarray): Angular velocity measurements [ang_vel_roll, ang_vel_pitch, ang_vel_yaw].
    - dt (float): Time step.

    Returns:
    - pose (np.ndarray): Estimated pose [position_x, position_y, position_z, orientation_roll, orientation_pitch, orientation_yaw].
    """
    t = prev_pose.translation()
    rpy = prev_pose.rotation().rpy()

    # init pose with prev pose 
    
    # Double integrate acceleration to get position
    t += 0.5 * accel * dt**2    # translation  pose[3:6] * dt + 
    
    # Euler integration for angular velocity to estimate orientation
    rpy += ang_vel * dt


    # Create a 4x4 identity matrix
    H = np.eye(4)

    # Assign the rotation matrix to the top-left 3x3 block
    H[:3, :3] = create_rotation_matrix_from_euler(rpy)

    # Assign the translation vector to the rightmost column
    H[:3, 3] = t

    return gtsam.Pose3(H)

def world_frame_cov(pose, sigma6):
    """Re-express a gtsam Pose3 marginal in the world frame.

    gtsam's 6x6 Pose3 covariance is in the BODY tangent frame (right
    perturbation), ordered [Rx Ry Rz Tx Ty Tz]. scoring.unc_radius reads the
    [3:5, 3:5] block as world x/y, which is only true when the body frame is
    level and north-aligned. Rotating both 3x3 blocks by the pose's rotation
    makes the block a genuine world x/y covariance: a pitching or rolling
    vehicle no longer leaks its vertical sigma into the horizontal one.
    """
    R = pose.rotation().matrix()
    T = np.zeros((6, 6))
    T[:3, :3] = R
    T[3:, 3:] = R
    return T @ np.asarray(sigma6) @ T.T


def flip_sigma(Sigma):
    """Flip between [Rx Ry Rz Tx Ty Tz] and [Tx Ty Tz Rx Ry Rz]"""
    order = [3, 4, 5, 0, 1, 2]
    Sigma = Sigma[np.ix_(order, order)] # Permute rows and columns
    return Sigma

def get_skew(phi):
    """Convert a 3D vector into a skew-symmetric matrix (hat operator)."""
    assert phi.shape == (3,), "Input must be a 3D vector [phi1 phi2 phi3]."
    return np.array([
        [0,        -phi[2],  phi[1]],
        [phi[2],    0,      -phi[0]],
        [-phi[1],  phi[0],   0]
    ])

def get_vee(mat):
    """Inverse of the hat operator: skew-symmetric matrix -> 3D vector."""
    return np.array([mat[2,1], mat[0,2], mat[1,0]])


def hat_se3(xi):
    """Returns the 4x4 matrix xi^wedge from a 6D twist vector"""
    rho = xi[3:]    # translation
    phi = xi[0:3]   # rotation
    return np.block([
        [get_skew(phi), rho.reshape(3,1)],
        [np.zeros((1, 4))]
    ])

def get_adjoint(pose:gtsam.Pose3):
    'in: gtsam pose'

    # Extract components
    xi = gtsam.Pose3.Logmap(pose)   # GTSAM returns [Rx Ry Rz Tx Ty Tz]

    R = pose.rotation().matrix()

    phi_bold = xi[0:3]       # rotation/twist part
    rho = xi[3:]            # translation part

    phi = np.linalg.norm(phi_bold)  # angle of rotation (scalar)


    # Compute Jacobian 3x3
    if phi < 1e-6:
        J = np.eye(3)
    else:
        a = phi_bold / phi              # axis of rotation (3x1)
        J = (np.sin(phi) / phi * np.eye(len(phi_bold))  
            + (1 - np.sin(phi) / phi) * a@a.T 
            + (1 - np.cos(phi))/ phi * get_skew(a))

    # Construct adjoint 6x6
    # |R   hat(Jp)R |
    # |0   R   |
    
    r1 = np.concatenate((R, get_skew(J@rho)@R), axis=1)
    r2 = np.concatenate((np.zeros_like(R), R), axis=1)
    adjoint = np.concatenate((r1, r2), axis=0)

    return adjoint


def get_between_cov(pose_ij, Sigma_ij, Sigma_ik, Sigma_ijik=None):
    '''Sigma [Tx Ty Tz Rx Ry Rz ]'''

    # Get adjoint Ad_{Tinv_{ij}}
    Ad = get_adjoint(pose_ij.inverse())

    # Assume not getting cross correlation terms
    Sigma_jk = Ad @ Sigma_ij @ Ad.T + Ad @ Sigma_ik @ Ad.T
    
    if Sigma_ijik is not None:
        Sigma_jk = Sigma_jk - Ad @ Sigma_ijik @ Ad.T - Ad @ Sigma_ijik.T @ Ad.T

    return Sigma_jk   
