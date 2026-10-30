import holoocean
import numpy as np
import matplotlib.pyplot as plt
from functools import partial

import gtsam
import gtsam_unstable
from gtsam.symbol_shorthand import A, B, J, K, V, W, Y, Z

from helpers import HfromRT, create_rotation_matrix_from_euler

np.set_printoptions(suppress=True)

class Agent:
    def __init__(self, name, poseKey, anchorKey, pose, path, global_frame_flag=False) -> None:

        ''''
        pose [x, y, z, roll, pitch, yaw]
        '''

        self.GLOBAL_FRAME = global_frame_flag

        # Indexing info
        self.name = name
        self.poseKeyStart = int(poseKey)
        self.prevposeKey = int(poseKey)
        self.poseKey = int(poseKey)
        self.anchorKey = int(anchorKey)

        # Pose info
        self.pose_vec = pose
        x0 = pose[0]; y0 = pose[1]; z0 = pose[2]
        rot = create_rotation_matrix_from_euler(pose[3:])
        H_init = HfromRT(rot, np.array([x0, y0, z0]))

        # Current pose
        self.pose_world_true = gtsam.Pose3(H_init)
        self.pose_local_true = gtsam.Pose3()
        self.pose_world_noisy = gtsam.Pose3(H_init)
        self.pose_local_noisy = gtsam.Pose3()
        self.anchor_true = self.pose_world_true

        # Prev pose
        self.prev_pose_world_true = self.pose_world_true
        self.prev_pose_local_true = self.pose_local_true
        self.prev_pose_world_noisy = self.pose_world_noisy
        self.prev_pose_local_noisy = self.pose_local_noisy

        self.prev_pose_world_noisy_comms = self.pose_world_noisy
        self.prev_pose_world_true_comms = self.pose_world_true

        # Path info
        self.path = path

        # For plotting
        self.x_true = {0:x0}
        self.y_true = {0:y0}
        self.x_noisy = {0:x0}
        self.y_noisy = {0:y0}
        self.comms_hist = {}

    def update_pose(self, pose_true, H_true, H_noisy, current_step):

        """
        take gtsam homogeneous transformation. His Hcurrent_wrt_prev
        
        """
        # Update poses
        self.pose_world_true = pose_true
        self.pose_local_true = self.prev_pose_local_true.compose(H_true)
        self.pose_world_noisy = self.prev_pose_world_noisy.compose(H_noisy)
        self.pose_local_noisy = self.prev_pose_local_noisy.compose(H_noisy)

        # For plotting
        self.x_true[current_step] = self.pose_world_true.translation()[0]
        self.y_true[current_step] = self.pose_world_true.translation()[1]      
        self.x_noisy[current_step] = self.pose_world_noisy.translation()[0]
        self.y_noisy[current_step] = self.pose_world_noisy.translation()[1]

        # Prev pose
        self.prev_pose_world_true = self.pose_world_true
        self.prev_pose_local_true = self.pose_local_true
        self.prev_pose_world_noisy = self.pose_world_noisy
        self.prev_pose_local_noisy = self.pose_local_noisy

    def update_prev_pose(self):
        self.prev_pose_world_noisy_comms = self.pose_world_noisy
        self.prev_pose_world_true_comms = self.pose_world_true

    def update_pose_key(self):
        self.prevposeKey = int(self.poseKey)
        self.poseKey = int(self.poseKey + 1)

    def update_comms(self, current_step, comms_type):
        self.comms_hist[current_step] = comms_type
