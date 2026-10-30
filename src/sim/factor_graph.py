import holoocean
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import cv2
from functools import partial

import gtsam
import gtsam_unstable
from gtsam.symbol_shorthand import L

from helpers import getHBetween, addVectorNoise, getBearing3D
from error import error_bearing, error_depth, error_depth_anchor, error_azi_elv_anchor, error_interbearing, error_fullpose_anchor, error_between, error_gps, error_unary_bearing, error_unary_bearing_anchor, error_landmark, error_landmark_anchor
from agent import Agent

np.set_printoptions(suppress=True)
plt.rcParams.update({'font.size': 15})

class FactorGraph:
    def __init__(self) -> None:
        self.ADD_NOISE = True
        self.GLOBAL_FRAME = False
        self.PARTIAL_BEARING = False
        self.ANCHOR_PRIORS_ALL = True
        self.LANDMARKS = True

        # Setup noise models
        self.std_pose = np.array([np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0), 0.02, 0.02, 0.01]) #np.array([0.02, 0.02, 0.02, np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0)])
        self.std_comms = np.array([np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0), 0.5, 0.5, 0.5])
        self.std_landmark = np.array([np.deg2rad(1), np.deg2rad(1), 2]) #np.array([0.02, 0.02, 0.01])
        self.std_orientation = np.array([np.deg2rad(1), np.deg2rad(1), np.deg2rad(1)])
        self.std_bearing = np.array([np.deg2rad(10.0), np.deg2rad(15.0)]) #
        self.std_depth = 0.5 # 0.01
        self.std_range = 3.0

        self.INIT_COV = None   # 6x6 [Rx Ry Rz Tx Ty Tz]; overrides INIT_NOISE when set
        self.rebuild_noise_models()

        # Setup gtsam
        self.isam = gtsam.ISAM2()
        self.graph = gtsam.NonlinearFactorGraph()
        self.initialEstimate = gtsam.Values()
            
        self.agents = {}
        self.t = []

        self.prev_result = None


    def rebuild_noise_models(self):
        """Derive every gtsam noise model from the std_* attributes. Call after
        changing any of them (calibrate_to_planner does), or the factors keep
        the sigmas the constructor saw."""
        if self.INIT_COV is not None:
            self.INIT_NOISE = gtsam.noiseModel.Gaussian.Covariance(np.asarray(self.INIT_COV, float))
        else:
            self.INIT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(10*np.array([np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0), 0.1, 0.1, 0.1]))
        self.POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_pose)
        self.LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_landmark)
        self.COMMS_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_comms)
        self.UNARY_BEARING_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_orientation)
        self.PARTIAL_BEARING_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_bearing) # azimuth and elevation
        self.DEPTH_NOISE = gtsam.noiseModel.Isotropic.Sigma(1, self.std_depth)
        # self.INTERBEARING_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, self.std_bearing)
        self.RANGE_NOISE = gtsam.noiseModel.Isotropic.Sigma(1, self.std_range)

    # Cross-track growth in this graph is governed by the compass, not by
    # std_pose[4]: heading error times the 5 m step is what walks the estimate
    # sideways. Measured with the scratch probe (straight track, world-frame
    # marginal, linear fit of variance vs arc): q_cross ~= 1.5e-3 m^2/m per
    # (1 deg compass)^2, and std_pose[4] in 0..0.075 m or the odometry rotation
    # sigma in 0.05..1 deg move it by <2x. Along-track growth IS std_pose[3]
    # (q_along = std_pose[3]^2 / node spacing, exact).
    Q_CROSS_PER_DEG2 = 1.5e-3

    def calibrate_to_planner(self, cfg, sigma0, node_spacing_m):
        """Give this graph the planner's ruler (landmark_planning config/main.yaml).

        The planner grows a 2-D covariance as a random walk with variance rates
        Q_DIR_PER_M = dir_uncertainty_per_meter^2 * 100 along the heading and
        that / maj_min_unc_ratio^2 across it (covariance.jl growth_covariance,
        config.jl UNC_CALIB_SEG_M = 100). Its initial covariance is landmark
        1's (`sigma0`). Landmark fixes carry landmark_sensor_noise along the
        line of sight and landmark_sensor_noise*bearing_noise_ratio across it,
        in metres; a comm transfers position with comm_sensor_noise (scaled per
        call by the driver with the link weight). Without this the two models
        differ by ~300x in variance and unc_ratio_planner_over_sim compares
        two different rulers.
        """
        dir_rate = float(cfg.get("dir_uncertainty_per_meter", 0.05))
        ratio = float(cfg.get("maj_min_unc_ratio", 3))
        q_along = dir_rate ** 2 * 100.0
        q_cross = q_along / ratio ** 2
        self.std_pose[3] = np.sqrt(q_along * node_spacing_m)
        compass = np.deg2rad(np.sqrt(q_cross / self.Q_CROSS_PER_DEG2))
        self.std_orientation = np.array([compass, compass, compass])
        self.std_comms[:3] = compass                      # relative heading, two compasses
        self.std_comms[3:] = float(cfg.get("comm_sensor_noise", 0.038))
        # The planner fuses a landmark ONCE per waypoint (100 m hex nodes in
        # the discrete stage), so its landmark_sensor_noise is the noise of a
        # whole pass. This graph adds one factor per node while in range --
        # about 2*visibility_range/node_spacing sightings per pass, which
        # average down by the square root of that count. Scale the planner's
        # per-pass number back up to a per-sighting one so the two passes
        # carry the same information.
        vis = float(cfg.get("visibility_range", 100.0))
        n_sight = 2.0 * vis / node_spacing_m
        lm = float(cfg.get("landmark_sensor_noise", 0.038)) * np.sqrt(n_sight)
        perp = lm * float(cfg.get("bearing_noise_ratio", 2.2))
        # The planner's cross-LOS noise is a distance; the factor wants an
        # angle. Convert at half the visibility range, the typical fix range.
        bearing = np.arctan2(perp, 0.5 * vis)
        self.std_landmark = np.array([bearing, bearing, lm])
        if sigma0 is not None:
            cov = np.zeros((6, 6))
            cov[:3, :3] = np.eye(3) * np.deg2rad(10.0) ** 2
            cov[3:5, 3:5] = np.asarray(sigma0, float)
            cov[5, 5] = self.std_depth ** 2
            self.INIT_COV = cov
        self.rebuild_noise_models()
        return dict(q_along=q_along, q_cross=q_cross, std_pose=self.std_pose.copy(),
                    compass_deg=np.degrees(compass), std_landmark=self.std_landmark.copy(),
                    std_comms=self.std_comms.copy())

    def initAgent(self, agent, mainagent_name):

        # Init prior, initial estimate, and anchors
        if self.GLOBAL_FRAME:
            self.graph.push_back(gtsam.PriorFactorPose3(agent.poseKey, agent.pose_world_true, self.INIT_NOISE))
            self.initialEstimate.insert(agent.poseKey, agent.pose_world_noisy)          
        else:
            self.graph.push_back(gtsam.PriorFactorPose3(agent.poseKey, agent.pose_local_noisy, self.INIT_NOISE))
            self.initialEstimate.insert(agent.poseKey, agent.pose_local_noisy)

            if self.ANCHOR_PRIORS_ALL:
                self.graph.push_back(gtsam.PriorFactorPose3(agent.anchorKey, agent.pose_world_true, self.INIT_NOISE))
            else:
                if agent.name==mainagent_name:
                    self.graph.push_back(gtsam.PriorFactorPose3(agent.anchorKey, agent.pose_world_true, self.INIT_NOISE))
            
            self.initialEstimate.insert(agent.anchorKey, agent.pose_world_true) # TODO noisy?

        self.agents[agent.name] = agent

    def initLandmark(self, landKey, landTrans, cov=None):
        # `cov` is the planner's 2x2 landmark position covariance
        # (scenario_landmarks.csv). When given, the landmark gets a prior with
        # it, which is what the planner's landmark_info models: an absolute fix
        # against a KNOWN landmark, S_total = S_sensor + lm.cov. Without the
        # prior the landmark is a free SLAM point -- it only ties sightings to
        # each other and bounds nothing absolutely, so the along-track drift
        # the planner expects a landmark pass to reset never resets.
        #
        # Insert the landmark initial estimate only if it's not already known
        # to this FactorGraph (either in the pending initialEstimate or in
        # the previous result stored in self.prev_result). Re-inserting a
        # key that ISAM already knows causes a runtime error.
        try:
            already_pending = self.initialEstimate.exists(landKey)
        except Exception:
            already_pending = False

        already_in_result = False
        if self.prev_result is not None:
            try:
                already_in_result = self.prev_result.exists(landKey)
            except Exception:
                already_in_result = False

        if already_pending or already_in_result:
            # No-op: ISAM already has this key or it's already staged for
            # insertion in the current update. Skip to avoid duplicate key.
            # Helpful debug when tracking propagation.
            # print(f"initLandmark: skipping insert for existing key {landKey}")
            return

        self.initialEstimate.insert(landKey, gtsam.Point3(landTrans))
        if cov is not None:
            # The plan is 2D and the field is lifted onto one depth plane, so
            # z gets the depth sensor's sigma rather than a planner number.
            cov3 = np.zeros((3, 3))
            cov3[:2, :2] = np.asarray(cov, dtype=float)
            cov3[2, 2] = self.std_depth ** 2
            self.graph.add(gtsam.PriorFactorPoint3(landKey, gtsam.Point3(landTrans),
                                                   gtsam.noiseModel.Gaussian.Covariance(cov3)))

    def addNodes(self, agent_name, nodes, pose_noise=None, add_orientation=True):
        """Append a run of nodes for one agent in one go.

        Each node is a dict(H_true, H_noisy, depth, ori, dets) as the driver
        records it; `dets` maps a landmark key to (azimuth, elevation, range).
        A support's nodes are buffered by the driver and only replayed here
        when a comm succeeds: the primary's graph must not contain support
        measurements it has not received. (Adding them as they happen lets a
        support's landmark fix flow backwards along its chain, across the
        PREVIOUS comm factor and forward into the primary -- the estimator
        probe measured a 1.39 -> 1.16 m drop before the comm that carried it.)
        Starting guesses: a node may carry `init`, the agent's OWN solved world
        pose for that node (from its own graph, as the driver records it). Use
        it. Chaining the raw noisy odometry from the last solved key instead --
        the only alternative, since ISAM2 has not seen the intermediate keys --
        puts the guess 22-263 m off over a 160-node backlog (1 deg of rotation
        noise per step random-walks the heading ~13 deg), and ISAM2's single
        correction step per update then leaves the delivered chain wrong for
        several solves; the comm factor drags the primary 14-35 m with it.
        The chain is kept only as the fallback when no `init` is carried, or in
        the anchor frame when the anchor is not solved yet.
        """
        agent = self.agents[agent_name]
        if self.prev_result is not None and self.prev_result.exists(agent.poseKey):
            pose = self.prev_result.atPose3(agent.poseKey)
        elif self.GLOBAL_FRAME:
            pose = agent.pose_world_noisy if self.ADD_NOISE else agent.pose_world_true
        else:
            pose = agent.pose_local_noisy if self.ADD_NOISE else agent.pose_local_true
        anchor = None
        if not self.GLOBAL_FRAME and self.prev_result is not None and self.prev_result.exists(agent.anchorKey):
            anchor = self.prev_result.atPose3(agent.anchorKey)
        for nd in nodes:
            agent.update_pose_key()
            pose = pose.compose(nd["H_noisy"] if self.ADD_NOISE else nd["H_true"])
            init = nd.get("init")
            if init is not None:
                if self.GLOBAL_FRAME:
                    pose = init
                elif anchor is not None:
                    pose = anchor.between(init)     # world -> this agent's local frame
            self.addOdometry(agent_name, nd["H_true"], nd["H_noisy"], pose_noise=pose_noise, init_pose=pose)
            if add_orientation:
                self.addDepthOrientation(agent_name, depth_meas=nd["depth"], orientation_meas=nd["ori"])
            for key, (azi, elev, rng) in nd["dets"].items():
                self.addLandmark(agent_name, key, azi, elev, rng)

    def addOdometry(self, agent_name, H_pose2_wrt_pose1_true, H_pose2_wrt_pose1_noisy, pose_noise=None, init_pose=None): 

        agent = self.agents[agent_name]

        # Pose and odometry 
        if self.ADD_NOISE:
            H_pose2_wrt_pose1 = H_pose2_wrt_pose1_noisy
        else:
            H_pose2_wrt_pose1 = H_pose2_wrt_pose1_true

        # Initial estimate: given (a replayed backlog, whose previous key ISAM2
        # has not solved yet), else composed with the previous estimate
        if init_pose is not None:
            pose = init_pose
        elif self.prev_result!= None and self.prev_result.exists(agent.prevposeKey):
            pose_prev = self.prev_result.atPose3(agent.prevposeKey)
            pose = pose_prev.compose(H_pose2_wrt_pose1)
        # Initial estimate from pure dead reckoning
        else:

            # Get noisy measurements
            if self.ADD_NOISE:
                if self.GLOBAL_FRAME:
                    pose = agent.pose_world_noisy
                else:
                    pose = agent.pose_local_noisy
            else:
                if self.GLOBAL_FRAME:
                    pose = agent.pose_world_true
                else:
                    pose = agent.pose_local_true

        # Odometry between factors and initial estimates
        self.initialEstimate.insert(agent.poseKey, pose)

        if pose_noise is None:
            self.graph.add(gtsam.BetweenFactorPose3(agent.prevposeKey, agent.poseKey, H_pose2_wrt_pose1, self.POSE_NOISE))
        else:
            self.graph.add(gtsam.BetweenFactorPose3(agent.prevposeKey, agent.poseKey, H_pose2_wrt_pose1, pose_noise))

    def addDepth(self, agent_name, depth_meas=None):
        agent = self.agents[agent_name]
        
        # Custom depth and unary bearing factors if not using full pose inter-agent
        if self.GLOBAL_FRAME:
            self.graph.add(gtsam.CustomFactor(self.DEPTH_NOISE, [agent.poseKey], partial(error_depth, depth_meas)))
        else:
            self.graph.add(gtsam.CustomFactor(self.DEPTH_NOISE, [agent.poseKey, agent.anchorKey], partial(error_depth_anchor, depth_meas)))

    def addOrientation(self, agent_name, orientation_meas=None):

        agent = self.agents[agent_name]

        # Custom depth and unary bearing factors if not using full pose inter-agent
        if self.GLOBAL_FRAME:
            self.graph.add(gtsam.CustomFactor(self.UNARY_BEARING_NOISE, [agent.poseKey], partial(error_unary_bearing, [orientation_meas])))
        else:
            self.graph.add(gtsam.CustomFactor(self.UNARY_BEARING_NOISE, [agent.poseKey, agent.anchorKey], partial(error_unary_bearing_anchor, [orientation_meas])))

    def addDepthOrientation(self, agent_name, depth_meas=None, orientation_meas=None): 

        agent = self.agents[agent_name]

        # Custom depth and unary bearing factors if not using full pose inter-agent
        if self.GLOBAL_FRAME:
            self.graph.add(gtsam.CustomFactor(self.DEPTH_NOISE, [agent.poseKey], partial(error_depth, depth_meas)))
            self.graph.add(gtsam.CustomFactor(self.UNARY_BEARING_NOISE, [agent.poseKey], partial(error_unary_bearing, [orientation_meas])))
        else:
            self.graph.add(gtsam.CustomFactor(self.DEPTH_NOISE, [agent.poseKey, agent.anchorKey], partial(error_depth_anchor, depth_meas)))
            self.graph.add(gtsam.CustomFactor(self.UNARY_BEARING_NOISE, [agent.poseKey, agent.anchorKey], partial(error_unary_bearing_anchor, [orientation_meas])))

    # def addLandmark(self, agent_name, landmark)
    def addInterAgent(self, main_agent_name, sender_name, states, meas_bearing=None, noise=None):
        """`noise` overrides COMMS_NOISE for the full relative-pose factor (the
        driver scales it with the planner's comm weight); it is ignored by the
        bearing-only variants."""

        sender = self.agents[sender_name]
        receiver = self.agents[main_agent_name]
        comms_noise = self.COMMS_NOISE if noise is None else noise

        pos1 = gtsam.Pose3(states[sender_name]['PoseSensor'])#sender.pose_world_true
        pos2 = gtsam.Pose3(states[main_agent_name]['PoseSensor']) #agent.pose_world_true

        # Get measurements
        if meas_bearing is None:
            meas_bearing = getBearing3D(pos1, pos2)

        sender_pose = gtsam.Pose3(states[sender_name]['PoseSensor'])  
        receiver_pose = gtsam.Pose3(states[main_agent_name]['PoseSensor'])  
        H_receiver_wrt_sender, H_receiver_wrt_sender_noisy = getHBetween(sender_pose, receiver_pose, self.std_comms)

        # Add noise to measurements
        if self.ADD_NOISE:
            # TODO compose noise correctly
            # meas_bearing = addVectorNoise(meas_bearing, np.deg2rad(0.0001))
            H_receiver_wrt_sender = H_receiver_wrt_sender_noisy

        # Choose type of factor
        if self.GLOBAL_FRAME:
            if self.PARTIAL_BEARING:
                self.graph.add(gtsam.CustomFactor(self.PARTIAL_BEARING_NOISE, [sender.poseKey, receiver.poseKey], partial(error_bearing, meas_bearing)))
            else:
                self.graph.add(gtsam.BetweenFactorPose3(sender.poseKey, receiver.poseKey, H_receiver_wrt_sender, comms_noise))

                # Range only, interbearing only
                # self.graph.add(gtsam.RangeFactorPose3(receiver.poseKey, sender.poseKey, receiver_pose.range(sender_pose), self.RANGE_NOISE))
                # self.graph.add(gtsam.CustomFactor(self.INTERBEARING_NOISE, [sender.poseKey, receiver.poseKey], partial(error_interbearing, [sender_pose.between(receiver_pose).rotation()])))
            
        else:
            if self.PARTIAL_BEARING:
                self.graph.add(gtsam.CustomFactor(self.PARTIAL_BEARING_NOISE, [sender.poseKey, receiver.poseKey, sender.anchorKey, receiver.anchorKey], partial(error_azi_elv_anchor, meas_bearing)))
            else:  
                print("custom fullpose")
                self.graph.add(gtsam.CustomFactor(comms_noise, [sender.poseKey, receiver.poseKey, sender.anchorKey, receiver.anchorKey], partial(error_fullpose_anchor, [H_receiver_wrt_sender])))    

    def addLandmark(self, agent_name, landKey, azi_meas, elev_meas, range_meas):
        agent = self.agents[agent_name]

        if self.GLOBAL_FRAME:
            self.graph.add(gtsam.CustomFactor(self.LANDMARK_NOISE, [agent.poseKey, landKey], partial(error_landmark, [azi_meas, elev_meas, range_meas])))
        else:
            self.graph.add(gtsam.CustomFactor(self.LANDMARK_NOISE, [agent.poseKey, landKey, agent.anchorKey], partial(error_landmark_anchor, [azi_meas, elev_meas, range_meas])))

    def check_pose(self, result, receiver_name, sender_name, states):
        print("\ncheck pose")
        print("sender: ", sender_name)
        sender = self.agents[sender_name]
        receiver = self.agents[receiver_name]

        # Get measurements
        pose_sender_meas = gtsam.Pose3(states[sender_name]['PoseSensor'])  

        # Get pred
        if result.exists(sender.anchorKey) and result.exists(receiver.anchorKey) and result.exists(sender.poseKey) and result.exists(receiver.poseKey):
            anchor_sender_pred = result.atPose3(sender.anchorKey)
            pose_sender_pred = result.atPose3(sender.poseKey)
            pose_sender_pred = anchor_sender_pred.compose(pose_sender_pred)

            print("posekey", sender.poseKey)
            print("anchor", anchor_sender_pred.translation())
            print("local pose", result.atPose3(sender.poseKey).translation())
            print("actual global pose", pose_sender_meas.translation())
            print("global pose meas", pose_sender_meas.translation())
            print("global pose pred", pose_sender_pred.translation())
    
    def check_landmarks(self, result, agent_name, landmarks_true, landmark_measurements, landmark_idx, states):
        agent = self.agents[agent_name]

        # Get measurements
        agent_pose_meas = gtsam.Pose3(states[agent_name]['PoseSensor'])
        landmark_trans_meas = landmarks_true[landmark_idx]
        trans_diff = agent_pose_meas.translation() - landmark_trans_meas

        # Calculated in landmark class
        phi_meas = landmark_measurements[0]
        theta_meas = landmark_measurements[1]
        r_meas = landmark_measurements[2]

        # Get pred
        if self.GLOBAL_FRAME:
            print("TODO")
            exit()
        else:
            if result.exists(agent.anchorKey) and result.exists(agent.poseKey) and result.exists(L(landmark_idx)):
                agent_anchor_pred = result.atPose3(agent.anchorKey)
                agent_pose_pred = agent_anchor_pred.compose(result.atPose3(agent.poseKey))
                landmark_trans_pred = result.atPoint3(L(landmark_idx))

                trans_diff = agent_pose_pred.translation() - landmark_trans_pred
                r_pred = np.linalg.norm(trans_diff)
                phi_pred = np.arctan2(trans_diff[1],trans_diff[0])# - agent_pose[2]   # azimuth
                theta_pred = np.arcsin(trans_diff[2]/r_pred)  

                print("\ncheck landmarks")
                print("agent: ", agent_name)
                print("posekey", agent.poseKey)
                print("pose global actual", agent_pose_meas.translation())
                print("pose global pred", agent_pose_pred.translation())
                print("landmark actual", landmark_trans_meas)
                print("landmark pred", landmark_trans_pred)
                print("pred", [phi_pred, theta_pred, r_pred])
                print("meas", [phi_meas, theta_meas, r_meas])


    def update_graph(self):
        # Solve graph
        # print(self.graph)
        self.isam.update(self.graph, self.initialEstimate)
        result = self.isam.calculateEstimate()

        self.prev_result = result

        return result

    def clear_graph(self):
        self.initialEstimate.clear()
        self.graph.resize(0)
    
    # plot_results lived here. It is gone: viz.py now owns every view (the live
    # 2D panel, the in-sim 3D overlay and the offline paper figures), rendering
    # from run_log.npz so a re-style costs no simulation time. The old version
    # was also unusable at this scale -- it hardcoded xlim(-10, 90) for an 80 m
    # sinusoid and its save path read a PNG nothing ever wrote.
    #
    # ponytail: the one thing worth salvaging was its azimuth uncertainty wedge
    # (variance of atan2 propagated through both agents' marginals, drawn as an
    # arc from the sender). Reinstate it in viz.py if the paper wants to show
    # what the inter-agent bearing factor actually constrains; it needs the
    # sender/receiver marginals recorded at each comms event, which the Recorder
    # does not currently keep.
