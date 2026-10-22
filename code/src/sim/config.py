"""Build the HoloOcean scenario for one `landmark_planning` run.

The scenario is derived from the plan, not invented: agent count, start
positions and start headings all come from the planner's control points.

Agent naming and the primary
----------------------------
The planner is 1-based and its PRIMARY IS THE LAST agent index; this repo's
main agent is `auv0`. So the mapping is deliberately not the identity:

    planner agent NA  ->  auv0   (primary / main_agent)
    planner agent 1   ->  auv1   (support)
    planner agent 2   ->  auv2   ...

holoocean/waypointtest.py maps these the other way round and is wrong for this
repo -- it would fly the support's plan on the vehicle whose estimate we score.

Vehicle and control
-------------------
`CougUV` under control scheme 1 (custom dynamics: the agent takes global-frame
accelerations), driven by `holoocean.fossen_dynamics.FossenInterface` running
its `depthHeadingAutopilot`. The dynamics/actuator/autopilot blocks below are a
tuned configuration lifted from waypointtest.py, not library defaults.

`ticks_per_sec` is 30 and should stay there: FossenInterface takes its control
period straight from this key (`sampleTime = 1/ticks_per_sec`), and every gain
in the `autopilot` block was tuned at 30 Hz. The old value here was 50, which
would silently detune the controller.
"""

import os
import copy

import numpy as np

import plan_io

# Positive down, metres. The plan is planar, so any depth is consistent with it;
# the stagger keeps agents that cross in plan view from sharing a point in space.
# Note this means inter-agent collision is not under test -- only agent vs obstacle.
BASE_DEPTH_M = 240.0
DEPTH_STAGGER_M = 5.0

# Close-up framing, for EVERY agent: [x, y, z] offset in the agent's own frame
# (x fwd, y LEFT, z UP) plus [roll, pitch, yaw]. This is the single definition
# of what a close-up looks like -- tune the shot here and every agent's panel
# moves together. Behind and well above, looking down on the vehicle: at a
# shallower angle the persistent waypoint markers (drawn at the agent's own
# depth) pile up into a magenta band across the horizon and clutter the shot.
#
# The primary used to be filmed by a ViewportCapture instead (HoloOcean's
# spectator chase camera, whose offset lives in the Unreal binary and can't be
# read out), so its panel could only ever be eyeball-matched to the supports'
# -- different pitch, different resolution. One rigidly-mounted RGBCamera per
# agent makes the framing identical by construction. The cost is that a rigid
# mount rotates 1:1 with the hull, so the vehicle sits dead centre all mission
# with no chase-cam swing, and only the horizon rolls.
#
# POSITIVE pitch looks DOWN for a mounted camera sensor. Measured, not assumed:
# with both agents held at the same x,y and 5 m apart in depth, a camera aimed
# at pitch -18 from 12 m behind / 4 m above saw only open water, and the same
# camera at +18 put the vehicle dead centre. An earlier value here was the whole
# bug -- 2 m BELOW the hull at pitch +10 pointed it at bare seabed.
#
# Those are the only two knobs: the mount position is DERIVED from them, so the
# camera always points at the hull. Setting position and pitch independently is
# how that old bug happened.
CLOSEUP_CAM_PITCH_DEG = 35.0     # 0 = level astern, 90 = straight down
CLOSEUP_CAM_RANGE_M = 5.5        # slant distance to the hull; bigger = zoomed out
_p = np.radians(CLOSEUP_CAM_PITCH_DEG)
CLOSEUP_CAM_LOC = [-CLOSEUP_CAM_RANGE_M * float(np.cos(_p)), 0.0,
                   CLOSEUP_CAM_RANGE_M * float(np.sin(_p))]
CLOSEUP_CAM_ROT = [0.0, CLOSEUP_CAM_PITCH_DEG, 0.0]
# 16:9 and exactly record_video.PANEL_WIDTH, so the composite never resamples.
CLOSEUP_CAM_RES = (1280, 720)


def _sensors(record=False):
    """Only sensors something actually reads.

    Dropped from the old config: both IMUSensors and the DVLSensor. Odometry
    is synthesized from PoseSensor rather than integrated from inertial data
    (see main.py), so those were pure tick cost -- and this loop runs ~20k
    ticks per mission instead of 82. The RGBCamera is back, but only under
    `record`.
    """
    sensors = [
        # Required by FossenInterface: must be named exactly "DynamicsSensor",
        # UseCOM true and UseRPY false, or check_scenario_configuration raises.
        {"sensor_type": "DynamicsSensor",
         "configuration": {"UseCOM": True, "UseRPY": False}},
        # Second copy for readback: index 17 is yaw in degrees, which is the
        # convention set_goal expects.
        {"sensor_type": "DynamicsSensor", "sensor_name": "DynamicsSensorRPY",
         "configuration": {"UseCOM": True, "UseRPY": True}},
        {"sensor_type": "PoseSensor", "location": [0, 0, 0]},
        {"sensor_type": "DepthSensor", "location": [0, 0, 0],
         "configuration": {"id": 0}},
        {"sensor_type": "VelocitySensor", "location": [0, 0, 0]},
        # Secondary, independent check on the geometric collision test -- it
        # also sees terrain, which the 2D polygon test cannot.
        {"sensor_type": "CollisionSensor", "location": [0, 0, 0]},
    ]
    if record:
        # Frames for the demo video (RECORD in main.py). Every agent, primary
        # included, gets the SAME camera at the SAME offset and resolution --
        # that identity is the point, see CLOSEUP_CAM_LOC above. Gated behind
        # `record` (only set when main.py's RECORD is on) so plain scoring and
        # Monte Carlo runs don't pay for a sensor nobody reads: 1280x720 per
        # agent is not cheap to render.
        sensors.append({"sensor_type": "RGBCamera",
                        "location": list(CLOSEUP_CAM_LOC),
                        "rotation": list(CLOSEUP_CAM_ROT),
                        "configuration": {"CaptureWidth": CLOSEUP_CAM_RES[0],
                                          "CaptureHeight": CLOSEUP_CAM_RES[1]}})
    return sensors


def _beacon(idx):
    return {"sensor_type": "AcousticBeaconSensor", "location": [0, 0, 0],
            "configuration": {"id": int(idx)}}


# Tuned CougUV configuration, from holoocean/waypointtest.py:83-124.
FOSSEN_BLOCKS = {
    "fossen_model": "torpedo",
    "control_mode": "depthHeadingAutopilot",
    "dynamics": {
        "T_yaw": 1,                   # yaw time constant (s)
    },
    "actuator": {
        "fin_area": 0.00697,          # one side of one fin (m^2)
        "CL_delta": 0.5,              # fin coefficient of lift
        "deltaMax_fin_deg": 15,       # max fin deflection (deg)
        "T_delta": 0.1,               # fin actuation time constant (s)
        "nMax": 1525,                 # max thruster rpm
        "T_n": 1.0,                   # thruster time constant (s)
        "D_prop": 0.14,               # propeller diameter (m)
    },
    "autopilot": {
        "depth": {
            "wn_d_z": 0.12, "Kp_z": 0.153, "T_z": 100,
            "Kp_theta": 39.78, "Kd_theta": 17.1, "Ki_theta": 0.5,
            "wn_d_theta": 0.25, "K_w": 0.0, "theta_max_deg": 15,
            "outer_loop_threshold": 2.91, "surge_threshold": 0.6,
        },
        "heading": {
            "wn_d": 0.4, "zeta_d": 1.0, "r_max": 0.87,
            "lam": 0.1, "phi_b": 0.1, "K_d": 0.5, "K_sigma": 0.05,
        },
    },
}


class HoloOceanConfig:
    """Scenario dict + the plan<->agent bookkeeping that goes with it."""

    def __init__(self, run, base_depth=BASE_DEPTH_M, depth_stagger=DEPTH_STAGGER_M,
                 ticks_per_sec=30, world="OpenWater", uncapped_fps=False, record=False):
        self.run = run
        self.ticks_per_sec = ticks_per_sec
        self.num_agents = run.num_agents

        # auv0 is the planner's primary; auv1.. are its supports in order.
        self._planner_idx = {"auv0": run.primary_idx}
        for k in range(1, self.num_agents):
            self._planner_idx[f"auv{k}"] = k - 1

        self.depths = {name: base_depth + depth_stagger * i
                       for i, name in enumerate(self.get_agent_names())}

        # OpenWater's own config declares env_min/env_max = +/-1008 m in x and y.
        # That is fine for the 1000 m scenarios but SHORTER THAN the missions
        # that matter: gauntlet, maze and behind_wall run to 1200 m and
        # long_sparse to 1400 m, which would put the vehicle outside the world's
        # octree bounds. Derive the extent from the plan instead of inheriting a
        # default that silently doesn't fit.
        lo, hi = self._plan_extent(run, pad=150.0)
        deepest = base_depth + depth_stagger * max(0, self.num_agents - 1)

        self.cfg = {
            "name": "landmark_planning_sim",
            "world": world,          # 1000-1400 m corridors need open water
            "package_name": "Ocean",
            "main_agent": "auv0",
            "ticks_per_sec": ticks_per_sec,
            # False uncaps the render rate for headless batching; matching
            # ticks_per_sec keeps a watchable run real-time.
            "frames_per_sec": False if uncapped_fps else ticks_per_sec,
            "env_min": [float(lo[0]), float(lo[1]), -float(deepest) - 200.0],
            "env_max": [float(hi[0]), float(hi[1]), 100.0],
            "octree_min": 0.02,
            "octree_max": 5,
            "agents": [],
        }

        for i, name in enumerate(self.get_agent_names()):
            ctrl = run.ctrls[self._planner_idx[name]]
            depth = self.depths[name]
            tangent = plan_io.spline_tangent(ctrl, 0.0)

            agent = {
                "agent_name": name,
                "agent_type": "CougUV",
                "sensors": _sensors(record=record) + [_beacon(i)],
                "control_scheme": 1,    # custom dynamics -- Fossen drives it
                "location": [float(ctrl[0][0]), float(ctrl[0][1]), -float(depth)],
                # Start pointing along the path. Starting at a fixed [0,0,0]
                # would spend the first metres in a spurious turn and pollute
                # the earliest covariance samples.
                "rotation": [0.0, 0.0, float(np.degrees(np.arctan2(tangent[1], tangent[0])))],
            }
            agent.update(copy.deepcopy(FOSSEN_BLOCKS))
            self.cfg["agents"].append(agent)

    @staticmethod
    def _plan_extent(run, pad=150.0):
        """Bounding box of everything in the scenario, padded."""
        pts = [run.path(i) for i in range(run.num_agents)]
        if len(run.landmarks):
            pts.append(run.landmarks)
        pts += [o.verts for o in run.obstacles]
        allp = np.vstack(pts)
        return allp.min(axis=0) - pad, allp.max(axis=0) + pad

    # ---- accessors (same contract the rest of the repo already expects) ----

    def get_cfg(self):
        return self.cfg

    def get_agent_names(self):
        return [f"auv{i}" for i in range(self.num_agents)]

    def get_agents_init(self):
        """{name: [x, y, z, roll, pitch, yaw]} -- z up, angles in degrees."""
        return {a["agent_name"]: np.array(a["location"] + a["rotation"], dtype=float)
                for a in self.cfg["agents"]}

    # ---- plan <-> agent mapping ----

    def planner_idx(self, name):
        """Index into run.ctrls for a HoloOcean agent name."""
        return self._planner_idx[name]

    def is_primary(self, name):
        return name == self.cfg["main_agent"]

    def depth(self, name):
        return self.depths[name]

    def waypoints(self, name, spacing_m):
        return self.run.waypoints(self.planner_idx(name), spacing_m)

    def planned_path(self, name, samples_per_seg=None):
        kwargs = {} if samples_per_seg is None else {"samples_per_seg": samples_per_seg}
        return self.run.path(self.planner_idx(name), **kwargs)


def _self_check(run_dir):
    """The mapping is the thing worth pinning: getting it backwards flies the
    support's plan on the agent whose estimate we score, and everything
    downstream still looks plausible."""
    run = plan_io.load_run(run_dir)
    hc = HoloOceanConfig(run)

    assert hc.planner_idx("auv0") == run.primary_idx, "auv0 must be the planner's primary"
    assert sorted(hc.planner_idx(n) for n in hc.get_agent_names()) == list(range(run.num_agents)), \
        "agent mapping is not a bijection onto the planner's agents"

    # auv0's plan must end at the scenario goal; a support's must not.
    assert np.allclose(hc.planned_path("auv0")[-1], run.goal_xy), "auv0 is not flying the goal path"
    for n in hc.get_agent_names()[1:]:
        assert not np.allclose(hc.planned_path(n)[-1], run.goal_xy), f"{n} duplicates the primary"

    from holoocean.fossen_dynamics.fossen_interface import check_scenario_configuration
    for n in hc.get_agent_names():
        check_scenario_configuration(hc.get_cfg(), n)   # raises if the Fossen block is wrong

    for a in hc.cfg["agents"]:
        start = run.ctrls[hc.planner_idx(a["agent_name"])][0]
        assert np.allclose(a["location"][:2], start), "start position does not match the plan"
        assert a["location"][2] < 0, "HoloOcean z is up; depth must be negative"

    print(f"agents   : {hc.get_agent_names()}")
    for n in hc.get_agent_names():
        a = hc.cfg["agents"][int(n[3:])]
        print(f"  {n}: planner agent {hc.planner_idx(n) + 1}"
              f"{' (primary)' if hc.is_primary(n) else ''}"
              f"  start={np.round(a['location'], 1).tolist()}"
              f"  yaw={a['rotation'][2]:.1f} deg")
    print(f"world    : {hc.cfg['world']}  ticks_per_sec={hc.cfg['ticks_per_sec']}")
    print("\nOK")


if __name__ == "__main__":
    import sys

    default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "..", "results", "paper", "montecarlo")
    _self_check(sys.argv[1] if len(sys.argv) > 1 else default)
