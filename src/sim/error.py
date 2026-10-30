import gtsam
import numpy as np
from typing import List, Optional
from helpers import getBearing3D

def get_unary_bearing_anchor(pose, anchor, measurement):

    # Compose to world frame
    pose = anchor.compose(pose)
    rot_pred = pose.rotation()

    # Get full relative pose meas
    rot_meas = measurement[0]

    # print("rot pred", rot_pred)
    # print("rot meas", rot_meas)

    error = gtsam.Rot3.Logmap(rot_pred.between(rot_meas))
    # print("between anchor error", error)

    return error

def error_unary_bearing_anchor(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    
    '''Interagent factor using bearing azimuth and elevation'''

    # azimuth and elevation
    pose = values.atPose3(this.keys()[0])
    anchor = values.atPose3(this.keys()[1])

    if jacobians is not None:
        eps = 1e-6

        # Six rows bc error state, six columns bc the state of this node is Pose3
        # Perturb relative to robot A pose key
        H0 = np.zeros((3, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseA_f = pose.compose(delta_step_forward)
            poseA_b = pose.compose(delta_step_backward)

            error_forward = get_unary_bearing_anchor(poseA_f, anchor, measurement)
            error_backward = get_unary_bearing_anchor(poseA_b, anchor, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        # Perturb relative to anchor key
        H1 = np.zeros((3, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            anchor_f = anchor.compose(delta_step_forward)
            anchor_b = anchor.compose(delta_step_backward)

            error_forward = get_unary_bearing_anchor(pose, anchor_f, measurement)
            error_backward = get_unary_bearing_anchor(pose, anchor_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H1[:,i] = hdot

        jacobians[0] = H0
        jacobians[1] = H1

    error = get_unary_bearing_anchor(pose, anchor, measurement)

    return error

def get_unary_bearing(pose, measurement):

    rot_pred = pose.rotation()

    # Get full relative pose meas
    rot_meas = measurement[0]

    # print("rot pred", rot_pred)
    # print("rot meas", rot_meas)

    error = gtsam.Rot3.Logmap(rot_pred.between(rot_meas))
    # print("between anchor error", error)

    return error

def error_unary_bearing(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    
    '''Interagent factor using bearing azimuth and elevation'''

    # azimuth and elevation
    key_pose = this.keys()[0]
    pose = values.atPose3(key_pose)

    if jacobians is not None:
        eps = 1e-6

        # Six rows bc error state, six columns bc the state of this node is Pose3
        # Perturb relative to robot A pose key
        H0 = np.zeros((3, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseA_f = pose.compose(delta_step_forward)
            poseA_b = pose.compose(delta_step_backward)

            error_forward = get_unary_bearing(poseA_f, measurement)
            error_backward = get_unary_bearing(poseA_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        jacobians[0] = H0

    error = get_unary_bearing(pose, measurement)

    return error

def get_between(poseA, poseB, measurement):

    # Get full relative pose pred
    rel_pose_pred = poseA.between(poseB)

    # Get full relative pose meas
    rel_pose_meas = measurement[0]

    error = gtsam.Pose3.Logmap(rel_pose_pred.between(rel_pose_meas))
    # print("between anchor error", error)

    return error

def error_between(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    
    '''Interagent factor using bearing azimuth and elevation'''

    # azimuth and elevation
    key_poseA = this.keys()[0]
    key_poseB = this.keys()[1]
    poseA = values.atPose3(key_poseA)
    poseB = values.atPose3(key_poseB)

    if jacobians is not None:
        eps = 1e-6

        # Six rows bc error state, six columns bc the state of this node is Pose3
        # Perturb relative to robot A pose key
        H0 = np.zeros((6, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseA_f = poseA.compose(delta_step_forward)
            poseA_b = poseA.compose(delta_step_backward)

            error_forward = get_between(poseA_f, poseB, measurement)
            error_backward = get_between(poseA_b, poseB, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        # Perturb relative to robot B pose key
        H1 = np.zeros((6, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseB_f = poseB.compose(delta_step_forward)
            poseB_b = poseB.compose(delta_step_backward)

            error_forward = get_between(poseA, poseB_f, measurement)
            error_backward = get_between(poseA, poseB_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H1[:,i] = hdot


        jacobians[0] = H0
        jacobians[1] = H1

    error = get_between(poseA, poseB, measurement)

    return error

def get_error_fullpose_anchor(poseA, poseB, anchorA, anchorB, measurement):

    # Compose to world frame
    poseA_world = anchorA.compose(poseA)
    poseB_world = anchorB.compose(poseB)

    # Get full relative pose pred
    rel_pose_pred = poseA_world.between(poseB_world)

    # Get full relative pose meas
    rel_pose_meas = measurement[0]
    
    error = gtsam.Pose3.Logmap(rel_pose_pred.between(rel_pose_meas))
    # error = gtsam.Pose3.Logmap(rel_pose_meas.between(rel_pose_pred))

    # print("between anchor error", error)

    return error

def error_fullpose_anchor(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    
    '''Interagent factor using bearing azimuth and elevation'''

    # azimuth and elevation
    poseA = values.atPose3(this.keys()[0])
    poseB = values.atPose3(this.keys()[1])
    anchorA = values.atPose3(this.keys()[2])
    anchorB = values.atPose3(this.keys()[3])

    if jacobians is not None:
        eps = 1e-6

        # Six rows bc error state, six columns bc the state of this node is Pose3
        # Perturb relative to robot A pose key
        H0 = np.zeros((6, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseA_f = poseA.compose(delta_step_forward)
            poseA_b = poseA.compose(delta_step_backward)

            error_forward = get_error_fullpose_anchor(poseA_f, poseB, anchorA, anchorB, measurement)
            error_backward = get_error_fullpose_anchor(poseA_b, poseB, anchorA, anchorB, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        # Perturb relative to robot B pose key
        H1 = np.zeros((6, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseB_f = poseB.compose(delta_step_forward)
            poseB_b = poseB.compose(delta_step_backward)

            error_forward = get_error_fullpose_anchor(poseA, poseB_f, anchorA, anchorB, measurement)
            error_backward = get_error_fullpose_anchor(poseA, poseB_b, anchorA, anchorB, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H1[:,i] = hdot

        # Perturb relative to anchor A pose key
        H2 = np.zeros((6, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            anchorA_f = anchorA.compose(delta_step_forward)
            anchorA_b = anchorA.compose(delta_step_backward)

            error_forward = get_error_fullpose_anchor(poseA, poseB, anchorA_f, anchorB, measurement)
            error_backward = get_error_fullpose_anchor(poseA, poseB, anchorA_b, anchorB, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H2[:,i] = hdot

        # Perturb relative to anchor B pose key
        H3 = np.zeros((6, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            anchorB_f = anchorB.compose(delta_step_forward)
            anchorB_b = anchorB.compose(delta_step_backward)

            error_forward = get_error_fullpose_anchor(poseA, poseB, anchorA, anchorB_f, measurement)
            error_backward = get_error_fullpose_anchor(poseA, poseB, anchorA, anchorB_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H3[:,i] = hdot

        jacobians[0] = H0
        jacobians[1] = H1
        jacobians[2] = H2
        jacobians[3] = H3

    error = get_error_fullpose_anchor(poseA, poseB, anchorA, anchorB, measurement)

    return error

def get_azi_elv_anchor(poseA, poseB, anchorA, anchorB, measurement):
    # compose to world frame
    poseA_world = anchorA.compose(poseA)
    poseB_world = anchorB.compose(poseB)

    error = get_error_bearing(poseA_world, poseB_world, measurement)

    return error

def error_azi_elv_anchor(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    
    '''Interagent factor using bearing azimuth and elevation'''

    # azimuth and elevation
    key_poseA = this.keys()[0]
    key_poseB = this.keys()[1]
    key_anchorA = this.keys()[2]
    key_anchorB = this.keys()[3]
    poseA = values.atPose3(key_poseA)
    poseB = values.atPose3(key_poseB)
    anchorA = values.atPose3(key_anchorA)
    anchorB = values.atPose3(key_anchorB)

    error = get_azi_elv_anchor(poseA, poseB, anchorA, anchorB, measurement)

    if jacobians is not None:
        # Two rows bc [phi, theta] from error state, six columns bc the state of this node is Pose3
        eps = 1e-6

        # Perturb relative to robot A pose key
        H0 = np.zeros((2, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseA_f = poseA.compose(delta_step_forward)
            poseA_b = poseA.compose(delta_step_backward)

            error_forward = get_azi_elv_anchor(poseA_f, poseB, anchorA, anchorB, measurement)
            error_backward = get_azi_elv_anchor(poseA_b, poseB, anchorA, anchorB, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        # Perturb relative to robot B pose key
        H1 = np.zeros((2, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseB_f = poseB.compose(delta_step_forward)
            poseB_b = poseB.compose(delta_step_backward)

            error_forward = get_azi_elv_anchor(poseA, poseB_f, anchorA, anchorB, measurement)
            error_backward = get_azi_elv_anchor(poseA, poseB_b, anchorA, anchorB, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H1[:,i] = hdot

        # Perturb relative to anchor A pose key
        H2 = np.zeros((2, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            anchorA_f = anchorA.compose(delta_step_forward)
            anchorA_b = anchorA.compose(delta_step_backward)

            error_forward = get_azi_elv_anchor(poseA, poseB, anchorA_f, anchorB, measurement)
            error_backward = get_azi_elv_anchor(poseA, poseB, anchorA_b, anchorB, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H2[:,i] = hdot

        # Perturb relative to anchor B pose key
        H3 = np.zeros((2, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            anchorB_f = anchorB.compose(delta_step_forward)
            anchorB_b = anchorB.compose(delta_step_backward)

            error_forward = get_azi_elv_anchor(poseA, poseB, anchorA, anchorB_f, measurement)
            error_backward = get_azi_elv_anchor(poseA, poseB, anchorA, anchorB_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H3[:,i] = hdot

        jacobians[0] = H0
        jacobians[1] = H1
        jacobians[2] = H2
        jacobians[3] = H3

    return error

def get_error_bearing(pos1, pos2, measurement):

    # Bearing error, compute circular difference (azimuth and elevation)
    # bearing = getBearing3D(pos1, pos2)
    # error = ((bearing - measurement + np.pi) % (2 * np.pi)) - np.pi

    phi_pred, theta_pred = getBearing3D(pos1, pos2)

    phi_pred = gtsam.Rot2(phi_pred)
    theta_pred = gtsam.Rot2(theta_pred)
    phi_meas = gtsam.Rot2(measurement[0])
    theta_meas = gtsam.Rot2(measurement[1])
    
    # Get error
    phi_error = gtsam.Rot2.Logmap(phi_pred.between(phi_meas))
    theta_error = gtsam.Rot2.Logmap(theta_pred.between(theta_meas))
    
    error = np.array([phi_error, theta_error]).flatten()

    return error


def error_bearing(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    """Depth Factor error function
    :param measurement: Landmark measurement, to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    key1 = this.keys()[0]
    key2 = this.keys()[1]
    pos1 = values.atPose3(key1)
    pos2 = values.atPose3(key2)

    if jacobians is not None:
        eps = 1e-6

        H0 = np.zeros((2, 6))

        # Perturb wrt agent 1 pose
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            # Perturb pose
            pos1_f = pos1.compose(delta_step_forward) 
            pos1_b = pos1.compose(delta_step_backward)

            # Compute error
            error_forward = get_error_bearing(pos1_f, pos2, measurement)
            error_backward = get_error_bearing(pos1_b, pos2, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)

            H0[:,i] = hdot

        jacobians[0] = H0

        H1 = np.zeros((2, 6))

        # Perturb wrt agent 2 pose
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            # Perturb pose
            pos2_f = pos2.compose(delta_step_forward)
            pos2_b = pos2.compose(delta_step_backward)

            # Compute error
            error_forward = get_error_bearing(pos1, pos2_f, measurement)
            error_backward = get_error_bearing(pos1, pos2_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)

            H1[:,i] = hdot

        jacobians[0] = H0
        jacobians[1] = H1

    error = get_error_bearing(pos1, pos2, measurement)

    return error

def error_gpsRelative(measurement: np.ndarray, HA_wrt_B:np.ndarray, this: gtsam.CustomFactor,
              values: gtsam.Values,
              jacobians: Optional[List[np.ndarray]]) -> float:
    """GPS Factor error function
    :param measurement: GPS measurement, to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    # print("Error HA_wrt_B")
    # print(HA_wrt_B)
    # Get agent pose
    key = this.keys()[0]
    HA_wrt_world = values.atPose3(key)
    HB_wrt_world = (HA_wrt_world.inverse().compose(HA_wrt_B)).inverse()
    estimate = HB_wrt_world

    # Three rows bc [x, y, z] from gps state, six columns bc the state of this node is Pose3
    H = np.zeros((3, 6))
    eps = 1e-6

    # Get Jacobians. Perturb in each direction of the delta vector one at a time.
    for i in range(6):
        delta_step = np.zeros(6)
        delta_step[i] = eps
        delta_step_forward = gtsam.Pose3.Expmap(delta_step)
        delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

        # Perturb relative to robot pose key
        HA_forward = HA_wrt_world.compose(delta_step_forward)
        HA_backward = HA_wrt_world.compose(delta_step_backward)

        HB_wrt_world_f = (HA_forward.inverse().compose(HA_wrt_B)).inverse()
        HB_wrt_world_b = (HA_backward.inverse().compose(HA_wrt_B)).inverse()

        h_forward = HB_wrt_world_f.translation()
        h_backward = HB_wrt_world_b.translation()
        error_forward = h_forward - measurement
        error_backward = h_backward - measurement
        hdot =  (error_forward - error_backward) / (2*eps)
        # hdot = np.reshape(hdot, (3,1))
        H[:,i] = hdot
    # print("\nestimate\n", estimate)
    # print("meas\n", measurement)

    # Convert to world frame, then to agent B frame
    # Get translational components only
    
    error = estimate.translation() - measurement
    # print("error\n", error)
    if jacobians is not None:
        # Error wrt agent A
        # jacobians[0] = np.array([[1, 0, 0, 0, 0, 0],
        #                          [0, 1, 0, 0, 0, 0],
        #                          [0, 0, 1, 0, 0, 0]])
        jacobians[0] = H

    return error


def error_gps(measurement: np.ndarray, this: gtsam.CustomFactor,
              values: gtsam.Values,
              jacobians: Optional[List[np.ndarray]]) -> float:
    """GPS Factor error function
    :param measurement: GPS measurement, to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """

    # Get agent pose
    estimate = values.atPose3(this.keys()[0])

    # Three rows bc [x, y, z] from gps state, six columns bc the state of this node is Pose3
    H = np.zeros((3, 6))
    eps = 1e-6

    # Get Jacobians. Perturb in each direction of the delta vector one at a time.
    for i in range(6):
        delta_step = np.zeros(6)
        delta_step[i] = eps
        delta_step_forward = gtsam.Pose3.Expmap(delta_step)
        delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

        q_exp_forward = estimate.compose(delta_step_forward)
        q_exp_backward = estimate.compose(delta_step_backward)
        h_forward = q_exp_forward.translation()
        h_backward = q_exp_backward.translation()
        error_forward = h_forward - measurement
        error_backward = h_backward - measurement
        hdot =  (error_forward - error_backward) / (2*eps)
        H[:,i] = hdot

    
    error = estimate.translation() - measurement

    if jacobians is not None:
        # Error wrt agent A
        jacobians[0] = H
 

    return error

def get_error_landmark(agent_pose, landmark_trans, measurement):

    # Pred info (received)
    # print("pred agent global", agent_pose.translation())
    # print('pred landmark', landmark_trans)
    trans_diff = agent_pose.translation() - landmark_trans
    r_pred = np.linalg.norm(trans_diff)
    phi_pred = np.arctan2(trans_diff[1], trans_diff[0])    # azimuth
    theta_pred = np.arcsin(trans_diff[2]/r_pred)  # elevation

    # print("pred error func", phi_pred, theta_pred, r_pred)
    # print("meas error func", measurement[0], measurement[1], measurement[2])

    # Map to manifold
    phi_pred = gtsam.Rot2(phi_pred)
    theta_pred = gtsam.Rot2(theta_pred)

    phi_meas = gtsam.Rot2(measurement[0])
    theta_meas = gtsam.Rot2(measurement[1])
    r_meas = measurement[2]


    # Get error
    phi_error = gtsam.Rot2.Logmap(phi_pred.between(phi_meas))
    theta_error = gtsam.Rot2.Logmap(theta_pred.between(theta_meas))
    range_error = r_pred - r_meas
    
    error = np.array([phi_error[0], theta_error[0], range_error]).flatten()

    # print("r pred", r_pred)
    # print("r meas", r_meas)
    # print(error)
    # exit()

    return error

def error_landmark(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    
    agent_pose = values.atPose3(this.keys()[0])
    landmark_trans = values.atPoint3(this.keys()[1])

    if jacobians is not None:
        eps = 1e-6

        # One row bc error state has three elements, six columns bc the state of this node is Pose3
        H0 = np.zeros((3, 6))

        # Perturb relative to robot pose key x, y, z, roll, pitch, yaw
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            pose_f = agent_pose.compose(delta_step_forward)
            pose_b = agent_pose.compose(delta_step_backward)

            error_forward = get_error_landmark(pose_f, landmark_trans, measurement)
            error_backward = get_error_landmark(pose_b, landmark_trans, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        # Perturb relative to landmark position x, y, z
        H1 = np.zeros((3, 3))
        for i in range(3):
            delta_step = np.zeros(3)
            delta_step[i] = eps
            delta_step_forward = delta_step
            delta_step_backward = -delta_step

            trans_f = landmark_trans + delta_step_forward
            trans_b = landmark_trans + delta_step_backward

            error_forward = get_error_landmark(agent_pose, trans_f, measurement)
            error_backward = get_error_landmark(agent_pose, trans_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H1[:,i] = hdot

        jacobians[0] = H0
        jacobians[1] = H1


    error = get_error_landmark(agent_pose, landmark_trans, measurement)
    return error
    
def get_error_landmark_anchor(agent_pose, landmark_trans, anchor, measurement):
    # print("\nerror lan anchor")

    # Compose to world frame
    agent_pose_pred = anchor.compose(agent_pose)

    # print("anchor pred lan", anchor.translation())
    # print("pose local pred lan", agent_pose.translation())
    # print("pose global pred lan", agent_pose_pred.translation())

    error = get_error_landmark(agent_pose_pred, landmark_trans, measurement)

    return error

def error_landmark_anchor(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    
    # print("poseKey", this.keys()[0])
    agent_pose = values.atPose3(this.keys()[0])
    landmark_trans = values.atPoint3(this.keys()[1])
    anchor = values.atPose3(this.keys()[2])

    if jacobians is not None:
        eps = 1e-6

        # One row bc error state has three elements, six columns bc the state of this node is Pose3
        H0 = np.zeros((3, 6))

        # Perturb relative to robot pose key
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            pose_f = agent_pose.compose(delta_step_forward)
            pose_b = agent_pose.compose(delta_step_backward)

            error_forward = get_error_landmark_anchor(pose_f, landmark_trans, anchor, measurement)
            error_backward = get_error_landmark_anchor(pose_b, landmark_trans, anchor, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        # Perturb relative to landmark position
        H1 = np.zeros((3, 3))
        for i in range(3):
            delta_step = np.zeros(3)
            delta_step[i] = eps
            
            trans_f = landmark_trans + delta_step
            trans_b = landmark_trans - delta_step

            error_forward = get_error_landmark_anchor(agent_pose, trans_f, anchor, measurement)
            error_backward = get_error_landmark_anchor(agent_pose, trans_b, anchor, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H1[:,i] = hdot

        # Perturb wrt anchor
        H2 = np.zeros((3, 6))
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            # Perturb the ANCHOR. This used to perturb agent_pose, which made
            # H2 a derivative evaluated at anchor := agent_pose*delta -- garbage
            # that let every landmark factor drag the anchor's yaw, with the
            # whole trajectory as its lever arm (hundreds of metres of error
            # while the marginal shrank). See estimator_probe.py's Jacobian check.
            anchor_f = anchor.compose(delta_step_forward)
            anchor_b = anchor.compose(delta_step_backward)

            error_forward = get_error_landmark_anchor(agent_pose, landmark_trans, anchor_f, measurement)
            error_backward = get_error_landmark_anchor(agent_pose, landmark_trans, anchor_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H2[:,i] = hdot

        jacobians[0] = H0
        jacobians[1] = H1
        jacobians[2] = H2


    error = get_error_landmark_anchor(agent_pose, landmark_trans, anchor, measurement)
    
    return error

def error_depth(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    """Depth Factor error function
    :param measurement: Landmark measurement, to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    pos = values.atPose3(this.keys()[0])

    # Depth error (z position). TODO 1D or 2D array?
    error = pos.translation()[2] - measurement[0]
    # print("depth pred", pos.translation()[2])
    # print("depth meas", pos.translation()[2])

    # print("depth error", error)

    # Jacobians
    if jacobians is not None:
        eps = 1e-6

        # Wrt pose key
        H0 = np.zeros((1, 6))

        # Get Jacobians. Perturb in each direction of the delta vector one at a time.
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            # Perturb pose
            q_exp_forward = pos.compose(delta_step_forward)
            q_exp_backward = pos.compose(delta_step_backward)

            # Extract depth
            h_forward = q_exp_forward.translation()[2]
            h_backward = q_exp_backward.translation()[2]

            # Compute error
            error_forward = h_forward - measurement[0]
            error_backward = h_backward - measurement[0]
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        jacobians[0] = H0

    return error

def get_error_interbearing(poseA_pred, poseB_pred, measurement):

    # Get relative orientation pred
    pose_rel_pred = poseA_pred.between(poseB_pred)
    ori_rel_pred = pose_rel_pred.rotation()

    # Get relative orientation meas
    ori_rel_meas = measurement[0]

    # Get error
    error = gtsam.Rot3.Logmap(ori_rel_pred.between(ori_rel_meas))

    return error 

def error_interbearing(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    
    
    key_poseA = this.keys()[0]
    key_poseB = this.keys()[1]
    poseA = values.atPose3(key_poseA)
    poseB = values.atPose3(key_poseB)

    if jacobians is not None:
        eps = 1e-6

        # One row bc error state has three elements, six columns bc the state of this node is Pose3
        H0 = np.zeros((3, 6))

        # Perturb relative to robot A pose key
        for i in range(3):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseA_f = poseA.compose(delta_step_forward)
            poseA_b = poseA.compose(delta_step_backward)

            error_forward = get_error_interbearing(poseA_f, poseB, measurement)
            error_backward = get_error_interbearing(poseA_b, poseB, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        # Perturb relative to robot B pose key x, y, theta
        H1 = np.zeros((3, 6))
        for i in range(3):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            poseB_f = poseB.compose(delta_step_forward)
            poseB_b = poseB.compose(delta_step_backward)

            error_forward = get_error_interbearing(poseA, poseB_f, measurement)
            error_backward = get_error_interbearing(poseA, poseB_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H1[:,i] = hdot

        jacobians[0] = H0
        jacobians[1] = H1


    error = get_error_interbearing(poseA, poseB, measurement)
    return error


def error_depth_anchor(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    """Depth Factor error function
    :param measurement: Landmark measurement, to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    keyPoseLocal = this.keys()[0]
    keyAnchor = this.keys()[1]
    pos_local = values.atPose3(keyPoseLocal)
    anchor = values.atPose3(keyAnchor)

    # H_agent_wrt_world = H_local_wrt_world H_agent_wrt_local
    pos_global = anchor.compose(pos_local).translation()

    # print("Depth local")
    # print("pred\n", pos_global[2])
    # print("actual\n", measurement[0])

    # Depth error (z position)
    error = pos_global[2] - measurement[0]
    # print("depth pred", pos_global[2])
    # print("depth meas", measurement[0])
    # print("depth error", error)

    # Jacobians
    if jacobians is not None:
        eps = 1e-6

        # Wrt pose key
        H0 = np.zeros((1, 6))

        # Get Jacobians. Perturb in each direction of the delta vector one at a time.
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            # Perturb pose
            pos_local_forward = pos_local.compose(delta_step_forward)
            pos_local_backward = pos_local.compose(delta_step_backward)

            # Compose to get world frame
            q_exp_forward = anchor.compose(pos_local_forward)
            q_exp_backward = anchor.compose(pos_local_backward)

            # Extract depth
            h_forward = q_exp_forward.translation()[2]
            h_backward = q_exp_backward.translation()[2]

            # Compute error
            error_forward = h_forward - measurement
            error_backward = h_backward - measurement
            hdot =  (error_forward - error_backward) / (2*eps)
            # hdot = np.reshape(hdot, (3,1))
            H0[:,i] = hdot

        # Wrt anchor key
        H1 = np.zeros((1, 6))

        # Get Jacobians. Perturb in each direction of the delta vector one at a time.
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            # Perturb anchor
            anchor_forward = anchor.compose(delta_step_forward)
            anchor_backward = anchor.compose(delta_step_backward)

            # Compose to get world frame
            q_exp_forward = anchor_forward.compose(pos_local)
            q_exp_backward = anchor_backward.compose(pos_local)

            # Extract depth
            h_forward = q_exp_forward.translation()[2]
            h_backward = q_exp_backward.translation()[2]

            # Compute error
            error_forward = h_forward - measurement
            error_backward = h_backward - measurement
            hdot =  (error_forward - error_backward) / (2*eps)
            # hdot = np.reshape(hdot, (3,1))
            H1[:,i] = hdot

        # jacobians[0] = np.array([[0, 0, 1, 0, 0, 0]])
        # jacobians[1] = np.array([[0, 0, 1, 0, 0, 0]])
            
        jacobians[0] = H0
        jacobians[1] = H1

    return error