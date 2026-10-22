"""Fly a `landmark_planning` plan in HoloOcean and score it.

The question this answers: with a real vehicle model, a real controller, and
the factor-graph estimate in the control loop, does the primary agent reach the
goal and stay out of the obstacles?

Three rates, on purpose
-----------------------
control     every tick        the Fossen autopilot integrates at 1/ticks_per_sec
estimator   every ~5 m of arc a pose node per tick would be a 20k-node graph
comms       every ~100 m      the planner's own comm_interval, so simulated
                              fusion events land where it assumed they would

Arc length, not wall-clock, is the synchronisation variable throughout -- that
is what the planner uses (src/planner/covariance.jl), so events stay comparable.
Distance is accumulated from ground truth even when the controller is steering
on the estimate: scheduling on the estimate would couple event timing to
estimator error and make runs incomparable.

Keys while running: space pause/resume | n step | v 2D panel | f fast | q quit+save
"""

import os
import sys

import numpy as np

np.set_printoptions(suppress=True)

import gtsam
from gtsam.symbol_shorthand import L

import holoocean
from holoocean.fossen_dynamics import FossenInterface

import plan_io
import scoring
import viz
from agent import Agent
from config import HoloOceanConfig
from controller import WaypointFollower
from factor_graph import FactorGraph
from helpers import getBearing3D, getHBetween, world_frame_cov
from landmarks import Landmarks

# ---------------------------------------------------------------------------
# Flags -- edited here, matching this repo's no-CLI-args convention
# ---------------------------------------------------------------------------

def _flag(name, default):
    """Env override for the handful of knobs an A/B run has to vary.

    Everything else stays edited-in-place, matching this repo's convention; the
    truth-vs-estimate comparison and the output tag need to be scriptable
    because the two runs must otherwise be byte-identical.
    """
    v = os.environ.get(name)
    if v is None:
        return default
    if isinstance(default, bool):
        return v.strip().lower() in ("1", "true", "yes", "on")
    return type(default)(v) if default is not None else v


_positional = [a for a in sys.argv[1:] if not a.startswith("--")]
WATCH = "--watch" in sys.argv[1:]  # opens the live viewport with sane defaults, see below

# The common case is `python main.py <run_dir>`; RUN_DIR= is still honored for
# scripted callers (monte_carlo.py, an A/B pair) that already set the env.
RUN_DIR = _positional[0] if _positional else os.environ.get("RUN_DIR", "")
ALGO = None                                  # None = infer, when the run has one planner
OUT_TAG = os.environ.get("OUT_TAG", "sim")   # subdirectory under RUN_DIR for this run

# The experiment. USE_TRUTH=True steers on ground truth (baseline: is the PATH
# flyable?); False steers on the factor-graph estimate (is it flyable when you
# only know where you are as well as the estimator does?). The delta between
# the two runs is the result.
USE_TRUTH = _flag("USE_TRUTH", False)

# --watch flips these together to the combination that actually works for
# watching live. An explicit env var still wins over either default.
HEADLESS = _flag("HEADLESS", not WATCH)   # no viewport, uncapped fps -- must not change the log
RECORD = _flag("RECORD", not WATCH)       # dump per-agent RGBCamera frames for a video
# Defaults off unconditionally, --watch included: the matplotlib instrument
# panel crashes on its own (see README's known issue) and isn't needed to
# watch -- the 3D viewport from --watch already IS the live view.
LIVE_2D = _flag("LIVE_2D", False)
FIGURES = _flag("FIGURES", True)  # skip for e.g. monte_carlo.py, which only
                                   # reads run_log.npz and makes its own figure
CHASE_CAM = False  # move_viewport() teleports choke on this world's dense kelp;
                    # HoloOcean's own default viewer camera runs smoothly instead

# SEED is threaded through so repeated runs of the same plan can be forced to
# differ -- Monte Carlo needs distinct noise draws per trial.
SEED = _flag("SEED", 1000)
np.random.seed(SEED)

# Estimator switches -- unchanged in meaning from the original driver.
ADD_NOISE = True
# World-frame poses. The anchor formulation (False) estimates a separate
# "map origin" pose per agent and composes world = anchor o local. Every
# measurement here constrains the COMPOSITION, so the split between the two
# halves is a gauge freedom that nothing ever shrinks (7.1 deg from the two
# 10 deg priors) -- and marginalCovariance(poseKey) of the LOCAL pose reports
# exactly that gauge: 0.125 m of fake sigma per metre from the start, in the
# cross-track and vertical directions. The start pose is known from the plan,
# so there is no unknown origin to estimate: estimate world poses directly.
GLOBAL_FRAME = True
# False: a comm is a full relative pose between the two vehicles (a
# BetweenFactorPose3 with std_comms), which is the planner's comm model -- an
# H = I position transfer with comm_sensor_noise (covariance.jl, ci_comm) plus
# a relative heading both compasses can supply. True: azimuth/elevation only,
# 10/15 deg, which at 60-250 m separation carries no position information at
# all, so the primary could never inherit a support's landmark fix.
PARTIAL_BEARING = False
ANCHOR_PRIORS_ALL = True
LANDMARKS = True
ADD_INTERAGENT = True
BEARING_OUTLIERS = False
ADD_ORIENTATION = True   # heading is real now that the autopilot flies it, so
                         # the orientation factors the old driver had commented
                         # out ("heading is not implemented in physics") apply.

NODE_SPACING_M = 5.0     # factor-graph node every N metres of arc
WAYPOINT_SPACING_M = 25.0
# Env-overridable for the same reason USE_TRUTH is: this A/B has to vary one
# thing and leave the rest of the run identical.
#
# 0 (the default) aims straight at the next waypoint, which is exactly what
# HoloOcean's own example.py does -- the controller stays the simulator's
# stock follower, because the planner is what is under test here. Setting it
# non-zero (25 is characterised) switches to lookahead-based LOS guidance
# (Fossen 2021). Run the pair to show the verdict does not hinge on the
# guidance law: `LOOKAHEAD_M=0 OUT_TAG=sim_wp` vs `LOOKAHEAD_M=25 OUT_TAG=sim_los`.
LOOKAHEAD_M = _flag("LOOKAHEAD_M", 0.0)
# Give the factor graph the planner's noise ruler (config/main.yaml:
# dir_uncertainty_per_meter, maj_min_unc_ratio, sigma0, landmark and comm
# noise) instead of FactorGraph's hard-coded constants. Off, the two models
# differ by ~300x in variance and unc_ratio_planner_over_sim is meaningless;
# env-overridable so that A/B is one variable.
CALIBRATE_TO_PLANNER = _flag("CALIBRATE_TO_PLANNER", True)
SPEED_RPM = 1300         # ~2.2 m/s on this vehicle
TICKS_PER_SEC = 30       # do not raise: FossenInterface takes its dt from this
TICK_BUDGET = _flag("TICK_BUDGET", 60000)
LIVE_EVERY = 4           # redraw the 2D panel every Nth node


# ---------------------------------------------------------------------------

class Keys:
    """Non-blocking keyboard control.

    Replaces the old WAIT_PLOT, which called plt.waitforbuttonpress() inside the
    loop and halted the physics. `from pynput import keyboard` was already
    imported by the previous driver and never used; this is what it was for.
    """

    def __init__(self, enabled=True):
        self.paused = self.quit = False
        self.step = 0
        self.toggle_2d = self.toggle_fast = False
        self._listener = None
        if not enabled:
            return
        try:
            from pynput import keyboard
        except Exception as e:                     # headless box without X: not fatal
            print(f"[keys] disabled ({e})")
            return

        def on_press(key):
            k = getattr(key, "char", None) or getattr(key, "name", "")
            if key == keyboard.Key.space:
                self.paused = not self.paused
                print(f"[keys] {'paused' if self.paused else 'running'}")
            elif k == "n":
                self.step += 1
            elif k == "v":
                self.toggle_2d = True
            elif k == "f":
                self.toggle_fast = True
            elif k == "q":
                self.quit = True
                print("[keys] quitting, writing log")

        self._listener = keyboard.Listener(on_press=on_press)
        self._listener.daemon = True
        self._listener.start()

    def blocked(self):
        """True while the user wants the sim held. Consumes one single-step."""
        if self.paused and self.step > 0:
            self.step -= 1
            return False
        return self.paused

    def stop(self):
        if self._listener is not None:
            self._listener.stop()


def make_graph(run=None):
    """One FactorGraph with the experiment switches applied.

    The switches must be identical on the multi-agent graph and on every
    per-agent dead-reckoning graph, or the estimates they exchange are not
    talking about the same thing.
    """
    fg = FactorGraph()
    fg.ADD_NOISE = ADD_NOISE
    fg.GLOBAL_FRAME = GLOBAL_FRAME
    fg.PARTIAL_BEARING = PARTIAL_BEARING
    fg.ANCHOR_PRIORS_ALL = ANCHOR_PRIORS_ALL
    fg.LANDMARKS = LANDMARKS
    if CALIBRATE_TO_PLANNER and run is not None:
        fg.calibrate_to_planner(run.cfg, run.sigma0, NODE_SPACING_M)
    return fg


def world_pose(fg, agent, result):
    """Recover a world pose from the anchor-node parameterisation."""
    if result is None or not result.exists(agent.poseKey):
        return None
    local = result.atPose3(agent.poseKey)
    if fg.GLOBAL_FRAME:
        return local
    anchor = result.atPose3(agent.anchorKey) if result.exists(agent.anchorKey) else agent.anchor_true
    return anchor.compose(local)


def main():
    if not RUN_DIR:
        sys.exit("usage: python main.py <landmark_planning results/<timestamp> dir> [--watch]\n"
                 "  .venv/bin/python main.py /path/to/results/2026-08-10_17-58-46")

    run = plan_io.load_run(RUN_DIR, ALGO)
    hc = HoloOceanConfig(run, ticks_per_sec=TICKS_PER_SEC, uncapped_fps=HEADLESS, record=RECORD)
    cfg = hc.get_cfg()
    names = hc.get_agent_names()
    main_name = cfg["main_agent"]
    num_agents = len(names)

    print(f"run      : {run.run_dir} [{run.algo}]")
    print(f"agents   : {names}  (auv0 = planner agent {run.primary_idx + 1}, primary)")
    print(f"planned  : {run.meta.get('continuous_primary_length') or run.meta.get('primary_length'):.1f} m, "
          f"predicted terminal unc {run.meta.get('primary_unc')}")
    print(f"obstacles: {len(run.obstacles)}   landmarks: {len(run.landmarks)}")
    print(f"control  : {'GROUND TRUTH (baseline)' if USE_TRUTH else 'FACTOR-GRAPH ESTIMATE'}\n")

    # A window is only wanted for --watch: HEADLESS=0 alone (as RECORD-only
    # runs use, to keep frame pacing real-time) doesn't mean "show a window
    # and listen to my keyboard" -- the popup steals focus and Keys' pynput
    # listener is a GLOBAL hook, so a stray keypress in another window (e.g.
    # 'v') was toggling this run's LiveView. Nothing reads the viewport itself
    # any more -- close-ups come from per-agent RGBCameras, which capture
    # whether or not the viewport renders.
    env = holoocean.make(scenario_cfg=cfg, show_viewport=WATCH)
    # Quality 0 (lowest) everywhere except a recording run: it is what the
    # close-up frames are rendered at, and scoring runs have no frames to
    # render, so only the video pays for it.
    env.set_render_quality(3 if RECORD else 0)
    fossen = FossenInterface(names, cfg, multi_agent=True)
    # Set the control mode exactly once: switching into an autopilot repeatedly
    # can leave its low-pass filter's initial position stale (known HoloOcean
    # issue). Only set_goal is called in the loop.
    for n in names:
        fossen.set_control_mode(n, "depthHeadingAutopilot")

    # ---- estimation stack (structure unchanged from the original driver) ----
    agents_init = hc.get_agents_init()
    multi = make_graph(run)
    if CALIBRATE_TO_PLANNER:
        print(f"noise    : calibrated to planner -- std_pose {np.round(multi.std_pose, 4)}, "
              f"compass {np.degrees(multi.std_orientation[2]):.2f} deg, "
              f"landmark {np.round(multi.std_landmark, 4)}, comms {np.round(multi.std_comms, 4)}")
    else:
        print("noise    : FactorGraph's shipped constants (CALIBRATE_TO_PLANNER=0)")
    agent_graphs = {}
    for idx, name in enumerate(names, start=1):
        pose_vec = agents_init[name]
        a = Agent(name, idx * 1e6, num_agents * 1e6 + idx * 1e6, pose_vec, None,
                  global_frame_flag=GLOBAL_FRAME)
        multi.initAgent(a, main_name)
        agent_graphs[name] = make_graph(run)
        agent_graphs[name].initAgent(Agent(name, 0, 1e6, pose_vec, None,
                                           global_frame_flag=GLOBAL_FRAME), name)

    L_class = Landmarks.from_run(run, -hc.depth(main_name))
    if LANDMARKS:
        for i, (x, y) in enumerate(run.landmarks):
            multi.initLandmark(L(i), np.array([x, y, -hc.depth(main_name)]),
                               cov=run.landmark_cov[i])
    visibility = float(run.cfg.get("visibility_range", 100.0))
    comm_interval = float(run.cfg.get("comm_interval", 100.0))
    # The planner's acoustic-link model (covariance.jl: comm_weight, ci_comm):
    # a comm checkpoint only fuses when the sigmoid weight on the vehicles'
    # separation clears comm_weight_min, and the transferred position carries
    # comm_sensor_noise inflated by 1/w. Fallbacks are config/main.yaml's
    # defaults. Fusing regardless of range (as the driver used to) credits the
    # primary with comms the plan itself never counted on.
    comm_cfg = dict(range=float(run.cfg.get("comm_range", 100.0)),
                    width=float(run.cfg.get("comm_width", 2.5)),
                    weight_min=float(run.cfg.get("comm_weight_min", 1e-4)),
                    sensor_noise=float(run.cfg.get("comm_sensor_noise", 0.038)))

    followers = {n: WaypointFollower(hc.waypoints(n, WAYPOINT_SPACING_M), hc.depth(n),
                                     speed_rpm=SPEED_RPM, update_spacing_m=NODE_SPACING_M,
                                     lookahead_m=LOOKAHEAD_M)
                 for n in names}

    rec = scoring.Recorder(run, hc)
    overlay = viz.SimOverlay(env, hc, run, enabled=not HEADLESS, chase=CHASE_CAM)
    overlay.draw_static()
    live = viz.LiveView(hc, run, enabled=LIVE_2D)
    # pynput's listener is a GLOBAL keyboard hook (not scoped to the HoloOcean
    # window), so it must stay off for an unattended RECORD-only run -- only
    # --watch or an explicit LIVE_2D actually want interactive key control.
    keys = Keys(enabled=WATCH or LIVE_2D)

    # One identically-mounted RGBCamera per agent (see config.py's _sensors),
    # so every close-up frames the vehicle the same way. Frame dirs are
    # per-agent so record_video.py can encode one close-up video per agent.
    frames_dirs = {n: os.path.join(RUN_DIR, OUT_TAG, f"frames_{n}") for n in names}
    if RECORD:
        for d in frames_dirs.values():
            os.makedirs(d, exist_ok=True)

    # ---- per-agent loop state ----
    arc = {n: 0.0 for n in names}
    last_node = {n: 0.0 for n in names}
    last_comm = {n: 0.0 for n in names}
    last_xy = {n: None for n in names}
    est_pose = {n: None for n in names}     # multi-agent fused belief (the primary's)
    own_est = {n: None for n in names}      # each agent's own dead-reckoning estimate
    belief_xy = {n: None for n in names}    # steering belief, dead-reckoned between fixes
    own_sigma = {}   # each support's own dead-reckoning marginal, for its uncertainty ellipse
    pending = {n: [] for n in names}   # a support's nodes not yet transmitted to the primary
    node_count = 0
    comm_segments = []

    dt = 1.0 / TICKS_PER_SEC

    states = env.tick()
    # PoseSensor reports in the SENSOR's frame -- NED in the IMU socket, NWU in
    # the COM socket. A silent NED read flips y and z and looks exactly like a
    # controller sign error, so pin it before trusting either source.
    p0 = np.array(states[main_name]["PoseSensor"])[:3, 3]
    d0 = np.array(states[main_name]["DynamicsSensor"])[6:9]
    assert np.allclose(p0, d0, atol=0.5), \
        f"PoseSensor {p0} disagrees with DynamicsSensor {d0} -- frame mismatch (NED vs NWU)"

    # Prime each autopilot before the loop. Goals are otherwise only issued when
    # a factor-graph node lands, nodes need travelled arc, arc needs motion and
    # motion needs a goal -- a standing start would deadlock at zero rpm.
    for n in names:
        s = states[n]
        goal = followers[n].update(np.array(s["PoseSensor"])[:2, 3],
                                   np.array(s["DynamicsSensor"])[3:5],
                                   float(np.array(s["DynamicsSensorRPY"])[17]))
        if goal is not None:
            fossen.set_goal(n, *goal)

    try:
        for tick in range(TICK_BUDGET):
            while keys.blocked() and not keys.quit:
                live.pause(0.05)
            if keys.quit or followers[main_name].done:
                break
            if keys.toggle_2d:
                keys.toggle_2d = False
                live = viz.LiveView(hc, run, enabled=not live.enabled)
            if keys.toggle_fast:
                keys.toggle_fast = False
                overlay.enabled = not overlay.enabled

            for n in names:
                env.act(n, fossen.update(n, states))
            states = env.tick()
            t = float(states["t"])

            # Every tick, not the ~5m estimator cadence -- a real-time video
            # needs a frame per tick (30/s) or it plays back as a slideshow.
            # BMP, not PNG: these are throwaway frames re-encoded to h264
            # within seconds by record_video.py, so PNG's compression is pure
            # cost with nothing to show for it -- measured at ~130 ms/frame
            # here (cProfile: 83% of total tick-loop time) vs ~5 ms/frame for
            # an uncompressed BMP ffmpeg reads exactly as natively.
            if RECORD:
                from PIL import Image
                for n in names:
                    if "RGBCamera" in states[n]:
                        img = np.ascontiguousarray(states[n]["RGBCamera"][:, :, :3][:, :, ::-1])
                        Image.fromarray(img).save(os.path.join(frames_dirs[n], f"{tick:06d}.bmp"))

            overlay.flush_dashes()

            new_nodes = []
            for n in names:
                s = states[n]
                truth = gtsam.Pose3(np.array(s["PoseSensor"]))
                xyz = truth.translation()
                xy = xyz[:2]
                heading = float(np.array(s["DynamicsSensorRPY"])[17])

                if last_xy[n] is not None:
                    arc[n] += float(np.linalg.norm(xy - last_xy[n]))
                    overlay.flown(n, last_xy[n], xy)
                last_xy[n] = xy

                rec.tick(t, n, xyz, collided=bool(np.array(s["CollisionSensor"])[0]))
                overlay.heading(n, xyz, followers[n].target_heading,
                                on_depth=abs(xyz[2] + hc.depth(n)) <= 2.0)

                # ---------------- control: every tick -------------------------
                # Steering runs at the full tick rate on a belief that is
                # dead-reckoned forward from the last factor-graph fix. Issuing
                # a heading only when a fix lands (every 5 m ~= 2.3 s) leaves the
                # autopilot chasing a 2.3 s-old command; the measured lag then
                # exceeds heading_rearm_deg, the re-aim latches off, and the
                # vehicle orbits the waypoint instead of capturing it. A real
                # AUV likewise steers continuously and only *corrects* the
                # belief when the navigation filter updates.
                vel_w = np.array(s["DynamicsSensor"])[3:5]
                if belief_xy[n] is None:
                    belief_xy[n] = xy if USE_TRUTH else xy.copy()
                else:
                    belief_xy[n] = belief_xy[n] + vel_w * dt
                steer_xy = xy if USE_TRUTH else belief_xy[n]

                before = followers[n].idx
                goal = followers[n].update(steer_xy, vel_w, heading)
                if followers[n].idx > before:
                    overlay.waypoint_reached(n, followers[n].waypoints[before])
                # Re-issued every tick because the markers are drawn with a
                # ~1-tick lifetime, which is what keeps the lit window sliding
                # with the vehicle instead of accumulating (see draw_waypoints).
                overlay.draw_waypoints(n, followers[n].waypoints, followers[n].idx)
                if goal is not None:
                    fossen.set_goal(n, *goal)

                # ---------------- estimator: every NODE_SPACING_M -------------
                if arc[n] - last_node[n] < NODE_SPACING_M:
                    continue
                last_node[n] = arc[n]
                node_count += 1
                new_nodes.append(n)

                fg_a = agent_graphs[n].agents[n]
                H_true, H_noisy = getHBetween(fg_a.prev_pose_world_true, truth,
                                              std=multi.std_pose)
                fg_a.update_pose_key()

                # Shape matters: error_depth_anchor returns `pos[2] - measurement[0]`,
                # and GTSAM needs that to be a 1-VECTOR. DepthSensor is shape (1,),
                # so the extra bracket makes measurement[0] a (1,) array. Flattening
                # here makes it a scalar and ISAM2 fails on the Eigen cast.
                depth_meas = np.array([s["DepthSensor"]])
                # Orientation is measured NOW, from the current pose. The
                # original driver read agent.pose_world_true here, which is the
                # PREVIOUS node's pose (update_pose runs further down) -- a
                # one-node-stale heading. At 1 m ticks on a gentle sinusoid that
                # was invisible; at 5 m nodes on a turning path it fights the
                # odometry and the heading drifts, which is the difference
                # between ~0.2 m and ~12 m of position error over 500 m.
                pose_for_ori = truth
                if ADD_NOISE:
                    depth_meas = depth_meas + np.random.normal(0, 0.01, 1)
                    nz = np.random.normal(np.zeros(3), multi.std_orientation)
                    pose_for_ori = pose_for_ori.compose(
                        gtsam.Pose3(gtsam.Rot3.RzRyRx(nz), gtsam.Point3(0, 0, 0)))
                ori_meas = pose_for_ori.rotation()

                agent_graphs[n].addOdometry(n, H_true, H_noisy,
                                            pose_noise=gtsam.noiseModel.Diagonal.Sigmas(multi.std_pose))
                # Every agent carries a compass and a depth sensor, so its OWN
                # dead-reckoning graph gets these too -- not just the primary's
                # multi-agent graph. Without them a support's onboard estimate is
                # pure odometry: heading random-walks and it ends 271 m from
                # truth over a 1 km mission (estimator_probe.py: 107 m at 500 m).
                # That also inflates the covariance it transmits at comms, which
                # is what the primary fuses. The original driver had this line
                # commented out because heading was not simulated; it is now.
                if ADD_ORIENTATION:
                    agent_graphs[n].addDepthOrientation(n, depth_meas=depth_meas,
                                                        orientation_meas=ori_meas)
                # EVERY agent looks for landmarks, not just the primary. A
                # support's fixes belong in its OWN dead-reckoning graph: that
                # is what it navigates on, and it is also the source of the
                # covariance it transmits at comms, so a tightened support
                # estimate reaches the primary through the compounded
                # between-factor that is already there. Detecting for the
                # primary only (as the original driver did) meant the plans
                # where the support is the one that passes close to a landmark
                # -- which is most of them -- entered the sim with none of the
                # fixes the planner budgeted for.
                dets = L_class.detect_landmarks(
                    xyz, std=multi.std_landmark, add_noise=ADD_NOISE,
                    max_range=visibility) if LANDMARKS else {}
                for key, (azi, elev, rng) in dets.items():
                    lk = L(key)
                    # initLandmark is idempotent (it checks initialEstimate and
                    # the previous ISAM2 result), so no guard needed here.
                    agent_graphs[n].initLandmark(
                        lk, np.array([*run.landmarks[key], -hc.depth(main_name)]),
                        cov=run.landmark_cov[key])
                    agent_graphs[n].addLandmark(n, lk, azi, elev, rng)
                    rec.landmark_event(tick, arc[n], n, key + 1, xy)
                    overlay.landmark_ray(xyz, run.landmarks[key], -hc.depth(main_name))

                fg_a.update_pose(truth, H_true, H_noisy, node_count)
                res_n = agent_graphs[n].update_graph()

                # A support navigates on its OWN dead-reckoning graph, which
                # updates every node. Steering it on `multi` instead would use
                # the primary's fused belief about it, which only refreshes at
                # comms (every 100 m) -- the support would wander between fixes
                # and corrupt the very comms geometry the primary depends on.
                own_est[n] = world_pose(agent_graphs[n], fg_a, res_n)

                if n != main_name:
                    marg = gtsam.Marginals(agent_graphs[n].isam.getFactorsUnsafe(), res_n)
                    # Rotated into the world frame: gtsam's marginal is in the
                    # body frame, and a body-x/y block read as world x/y picks
                    # up sin(pitch) * sigma_z every time the vehicle tilts.
                    own_sigma[n] = world_frame_cov(own_est[n], marg.marginalCovariance(fg_a.poseKey))
                agent_graphs[n].clear_graph()

                # The primary's cooperative graph gets EVERY agent's nodes --
                # the same odometry, depth/compass and landmark factors that
                # went into that agent's own graph, drawn once and used twice
                # (re-detecting would double the measurement count and re-roll
                # the noise). For a support this is what makes its landmark
                # fix reachable: the old driver forwarded only a compounded
                # relative-motion factor at each comm, which carries no
                # absolute information, so a support could tighten its own
                # estimate to 0.3 m and the primary would never hear of it.
                # The primary's own nodes go in now; a support's are queued
                # and delivered when a comm succeeds (see FactorGraph.addNodes
                # for why they must not go in as they happen). Each queued node
                # also carries the agent's OWN solved world pose (`init`) --
                # what an acoustic packet would carry alongside the
                # measurements -- and addNodes uses it as the starting guess.
                # Chaining the raw noisy odometry over a 160-node backlog put
                # that guess 22-263 m off and the primary's estimate 14-35 m
                # off for ~4 nodes right on the last bend (the looping trials of
                # the 2026-09-09 Monte Carlo).
                multi.agents[n].update_pose(truth, H_true, H_noisy, node_count)
                node = dict(H_true=H_true, H_noisy=H_noisy, depth=depth_meas, ori=ori_meas,
                            dets={L(key): v for key, v in dets.items()}, init=own_est[n])
                if n == main_name:
                    multi.addNodes(n, [node], pose_noise=gtsam.noiseModel.Diagonal.Sigmas(multi.std_pose),
                                   add_orientation=ADD_ORIENTATION)
                else:
                    pending[n].append(node)
                    # ---------- comms: every comm_interval of arc ------------
                    if ADD_INTERAGENT and arc[n] - last_comm[n] >= comm_interval:
                        last_comm[n] = arc[n]
                        _add_comms(multi, n, main_name, states, node_count, rec, arc,
                                   overlay, comm_segments, tick, comm_cfg, pending)

            # ------- solve the multi-agent graph, only when a node landed -----
            # Solving every tick would be ~14k redundant ISAM2 updates per
            # mission and would record node metrics at the tick rate rather
            # than the estimator rate.
            if new_nodes:
                result = multi.update_graph()
                multi.clear_graph()
                for n in names:
                    wp = world_pose(multi, multi.agents[n], result)
                    if wp is not None:
                        est_pose[n] = wp

                a = multi.agents[main_name]
                try:
                    marg = gtsam.Marginals(multi.isam.getFactorsUnsafe(), result)
                    sigma = world_frame_cov(est_pose[main_name], marg.marginalCovariance(a.poseKey))
                except Exception:
                    sigma = None

                for n in new_nodes:
                    s = states[n]
                    truth_xyz = gtsam.Pose3(np.array(s["PoseSensor"])).translation()

                    # The primary's belief is the cooperative multi-agent
                    # estimate -- that is the thing under test. A support's is
                    # its own dead-reckoning, which is what it would actually
                    # have onboard. The same belief is both recorded and steered
                    # on, so est_err means "how wrong was this agent about
                    # itself" for every agent.
                    belief = est_pose[n] if n == main_name else own_est[n]
                    est_xyz = belief.translation() if belief is not None else truth_xyz

                    rec.node(n, tick, arc[n], truth_xyz, est_xyz,
                             sigma if n == main_name else own_sigma.get(n),
                             followers[n].target_heading,
                             float(np.array(s["DynamicsSensorRPY"])[17]))
                    overlay.estimate(truth_xyz, est_xyz)

                    # A fix landed: snap the dead-reckoned belief back onto it.
                    # Steering itself happens every tick, above.
                    if belief is not None:
                        belief_xy[n] = belief.translation()[:2].copy()

                overlay.follow(gtsam.Pose3(np.array(states[main_name]["PoseSensor"])).translation(),
                               float(np.array(states[main_name]["DynamicsSensorRPY"])[17]))
                if live.enabled and node_count % LIVE_EVERY == 0:
                    live.update(rec, comm_segments[-num_agents:])

                if node_count % 40 == 0:
                    print(f"  t={t:7.1f}s arc={arc[main_name]:8.1f}m "
                          f"wp={followers[main_name].idx}/{len(followers[main_name].waypoints)} "
                          f"unc={scoring.unc_radius(sigma) if sigma is not None else float('nan'):.3f}")
    finally:
        keys.stop()
        for n in names:
            rec.skipped_waypoints[n] = followers[n].skipped
        out_dir = os.path.join(RUN_DIR, OUT_TAG)
        report = rec.finish(out_dir)
        scoring.print_report(report)
        print(f"\nwrote {out_dir}/sim_report.yaml and run_log.npz")
        if FIGURES:
            try:
                for p in viz.render_all(out_dir):
                    print("wrote", p)
            except Exception as e:
                print(f"figure rendering failed: {e}")
        live.close()
        env.__exit__(None, None, None)


def comm_weight(dist_m, comm_cfg):
    """The planner's link quality: sigmoid on separation, half-weight at
    comm_range, softness comm_width (covariance.jl: comm_weight)."""
    return 1.0 / (1.0 + np.exp((dist_m - comm_cfg["range"]) / comm_cfg["width"]))


def _add_comms(multi, sender, main_name, states, step, rec, arc,
               overlay, comm_segments, tick, comm_cfg, pending):
    """A neighbour transmits: every node it has recorded since the last
    successful comm (odometry, depth/compass, landmark fixes), then one
    inter-agent factor tying its current node to the primary's. With
    PARTIAL_BEARING=False that factor is a full relative pose (the planner's
    position transfer); with True, azimuth/elevation only.

    Returns False when the planner's range weight says the link is down; the
    backlog then stays queued and the checkpoint is not a comm event (matching
    the planner's own comm_events.csv, which only lists fusions that cleared
    comm_weight_min)."""
    pos1 = gtsam.Pose3(np.array(states[sender]["PoseSensor"]))
    pos2 = gtsam.Pose3(np.array(states[main_name]["PoseSensor"]))
    dist = float(np.linalg.norm(pos1.translation()[:2] - pos2.translation()[:2]))
    w = comm_weight(dist, comm_cfg)
    if w < comm_cfg["weight_min"]:
        print(f"  comm skipped at arc={arc[sender]:.0f}m: {sender}<->{main_name} "
              f"{dist:.0f} m apart, weight {w:.1e} < {comm_cfg['weight_min']:g}")
        return False

    s_agent = multi.agents[sender]
    s_agent.update_comms(step, "SENDING")
    multi.agents[main_name].update_comms(step, "RECEIVING")
    multi.addNodes(sender, pending[sender], pose_noise=gtsam.noiseModel.Diagonal.Sigmas(multi.std_pose),
                   add_orientation=ADD_ORIENTATION)
    n_sent = len(pending[sender])
    pending[sender].clear()

    # Position noise comm_sensor_noise / sqrt(w): the planner inflates the
    # transferred covariance by 1/w (ci_comm), so a marginal link is trusted
    # less. Rotation keeps std_comms' 1 deg -- both vehicles carry a compass.
    sigma_t = comm_cfg["sensor_noise"] / np.sqrt(w)
    comm_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([*multi.std_comms[:3], sigma_t, sigma_t, sigma_t]))

    bearing = getBearing3D(pos1, pos2)
    if ADD_NOISE:
        noise = np.random.normal([0, 0], multi.std_bearing)
        if BEARING_OUTLIERS and np.random.randint(0, 20) == 1:
            noise = np.random.uniform(np.deg2rad(40), np.deg2rad(120), 2)
        phi = gtsam.Rot2.Logmap(gtsam.Rot2(bearing[0]).compose(gtsam.Rot2(noise[0])))
        theta = gtsam.Rot2.Logmap(gtsam.Rot2(bearing[1]).compose(gtsam.Rot2(noise[1])))
        if phi < 0:
            phi += 2 * np.pi
        bearing = np.array([float(phi), float(theta)])

    multi.addInterAgent(main_name, sender, states, meas_bearing=bearing, noise=comm_noise)
    print(f"  comm at arc={arc[sender]:.0f}m: {sender}->{main_name} {dist:.0f} m apart, weight {w:.3f}, "
          f"{n_sent} nodes delivered")
    rec.comm_event(tick, arc[main_name], sender, main_name, pos1.translation(), pos2.translation())
    comm_segments.append((pos1.translation(), pos2.translation()))
    overlay.comm(pos1.translation(), pos2.translation())
    return True


if __name__ == "__main__":
    main()
