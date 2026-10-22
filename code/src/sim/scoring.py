"""Record a run and score it.

Two artifacts, because they answer to different readers:

  sim_report.yaml  the verdict -- did the primary reach the goal, did it hit
                   anything, and did the factor graph's uncertainty match what
                   the planner predicted.
  run_log.npz      everything needed to draw the figures. Written so that
                   re-styling a figure never costs another 20-minute run;
                   viz.py reads this and never touches the simulator.

Uncertainty is reported as `det(Sigma_xy)^0.25`, matching `unc_radius` in the
planner (src/planner/covariance.jl) so `terminal_unc_m` is directly comparable to
`primary_unc` in the planner's own results.yaml. GTSAM orders a Pose3 marginal
[Rx Ry Rz Tx Ty Tz], so the planar position block is [3:5, 3:5] -- not [0:2].
"""

import os
from collections import defaultdict

import numpy as np

import plan_io

GOAL_TOL_M = 10.0


def unc_radius(sigma_pose6):
    """det(Sigma_xy)^0.25 from a GTSAM 6x6 Pose3 marginal.

    `sigma_pose6` may be an all-NaN placeholder (no marginal available for
    this node) -- return NaN rather than letting np.linalg.det warn on every
    such node.
    """
    xy = np.asarray(sigma_pose6)[3:5, 3:5]
    if not np.all(np.isfinite(xy)):
        return float("nan")
    return float(max(np.linalg.det(xy), 0.0) ** 0.25)


def _dump_yaml(obj, path):
    """Flat one-level YAML, mirroring what the planner's results.yaml looks like.

    Deliberately not PyYAML: this writes one nested block of scalars and lists,
    and plan_io.read_yaml already reads that dialect back."""
    def fmt(v):
        if isinstance(v, (bool, np.bool_)):
            return "true" if v else "false"
        if isinstance(v, (list, tuple, np.ndarray)):
            return "[" + ", ".join(fmt(x) for x in v) + "]"
        if isinstance(v, (float, np.floating)):
            return "null" if not np.isfinite(v) else f"{float(v):.6g}"
        if isinstance(v, (int, np.integer)):
            return str(int(v))
        return "null" if v is None else str(v)

    with open(path, "w") as f:
        for section, body in obj.items():
            f.write(f"{section}:\n")
            for k, v in body.items():
                f.write(f"  {k}: {fmt(v)}\n")


class Recorder:
    """Accumulates the run, then scores it.

    `tick` is the cheap per-tick path/collision record; `node` is called only
    when the factor graph produces a new estimate.
    """

    def __init__(self, run, hc, goal_tol=GOAL_TOL_M):
        self.run = run
        self.hc = hc
        self.goal_tol = goal_tol
        self.agent_radius = float(run.cfg.get("agent_radius", 0.0))
        self.primary = hc.cfg["main_agent"]

        self.ref = {n: hc.planned_path(n) for n in hc.get_agent_names()}
        self.track = defaultdict(list)      # per tick: [t, x, y, z]
        self.pen = defaultdict(list)        # per tick: geometric penetration (r_a = 0)
        self.hull_pen = defaultdict(list)   # per tick: with the agent radius
        self.collisions = defaultdict(int)  # CollisionSensor hits

        self.nodes = defaultdict(list)      # per FG node: dict of scalars/arrays
        self.comm_events = []
        self.landmark_events = []
        self.skipped_waypoints = {}

    # ---- accumulation -------------------------------------------------

    def tick(self, t, name, xyz, collided=False):
        x, y = float(xyz[0]), float(xyz[1])
        self.track[name].append([t, x, y, float(xyz[2])])
        worst = worst_hull = 0.0
        for obs in self.run.obstacles:
            worst = max(worst, plan_io.obstacle_penetration(x, y, obs))
            worst_hull = max(worst_hull, plan_io.obstacle_penetration(x, y, obs, self.agent_radius))
        self.pen[name].append(worst)
        self.hull_pen[name].append(worst_hull)
        if collided:
            self.collisions[name] += 1

    def node(self, name, tick, arc, truth_xyz, est_xyz, sigma6, heading_cmd, heading_act):
        self.nodes[name].append(dict(
            tick=int(tick),
            arc=float(arc),
            truth=np.asarray(truth_xyz, float),
            est=np.asarray(est_xyz, float),
            sigma=np.asarray(sigma6, float) if sigma6 is not None else np.full((6, 6), np.nan),
            unc=unc_radius(sigma6) if sigma6 is not None else np.nan,
            xtrack=plan_io.cross_track_error(truth_xyz, self.ref[name]),
            est_err=float(np.linalg.norm(np.asarray(truth_xyz)[:2] - np.asarray(est_xyz)[:2])),
            heading_cmd=float(heading_cmd), heading_act=float(heading_act),
        ))

    def comm_event(self, tick, arc, sender, receiver, sender_xy, receiver_xy):
        self.comm_events.append([float(arc), *map(float, sender_xy[:2]), *map(float, receiver_xy[:2]),
                                 float(tick)])

    def landmark_event(self, tick, arc, name, land_id, agent_xy):
        self.landmark_events.append([float(arc), float(land_id), *map(float, agent_xy[:2]), float(tick)])

    # ---- scoring ------------------------------------------------------

    def _arrays(self, name):
        n = self.nodes[name]
        if not n:
            return {}
        return {k: np.array([d[k] for d in n]) for k in n[0]}

    def report(self):
        p = self.primary
        track = np.array(self.track[p]) if self.track[p] else np.zeros((0, 4))
        nodes = self._arrays(p)
        pen = np.array(self.pen[p]) if self.pen[p] else np.zeros(1)
        hull = np.array(self.hull_pen[p]) if self.hull_pen[p] else np.zeros(1)

        final_xy = track[-1, 1:3] if len(track) else np.full(2, np.nan)
        goal_dist = float(np.linalg.norm(final_xy - self.run.goal_xy))
        breach = np.flatnonzero(pen > 0)
        flown = plan_io.arclength(track[:, 1:3])[-1] if len(track) > 1 else 0.0

        planned_unc = self.run.meta.get("primary_unc")
        term_unc = float(nodes["unc"][-1]) if len(nodes.get("unc", [])) else np.nan

        out = {
            "result": {
                "reached_goal": bool(goal_dist <= self.goal_tol),
                "goal_distance_m": goal_dist,
                "max_penetration_m": float(pen.max()),
                "max_hull_penetration_m": float(hull.max()),
                "first_breach_m": float(plan_io.arclength(track[:breach[0] + 1, 1:3])[-1]) if len(breach) else None,
                "collision_sensor_hits": int(self.collisions[p]),
                "skipped_waypoints": self.skipped_waypoints.get(p, []),
            },
            "tracking": {
                "rms_cross_track_m": float(np.sqrt(np.mean(nodes["xtrack"] ** 2))) if len(nodes.get("xtrack", [])) else np.nan,
                "max_cross_track_m": float(nodes["xtrack"].max()) if len(nodes.get("xtrack", [])) else np.nan,
                "arc_length_m": float(flown),
                "planned_length_m": self.run.meta.get("continuous_primary_length") or self.run.meta.get("primary_length"),
                "ticks": int(len(track)),
            },
            "estimation": {
                "terminal_est_error_m": float(nodes["est_err"][-1]) if len(nodes.get("est_err", [])) else np.nan,
                "rms_est_error_m": float(np.sqrt(np.mean(nodes["est_err"] ** 2))) if len(nodes.get("est_err", [])) else np.nan,
                "terminal_unc_m": term_unc,
                "planner_predicted_unc_m": planned_unc,
                # >1 means the factor graph ended up more confident than the
                # planner's covariance model predicted -- i.e. the model is
                # pessimistic. <1 means it is optimistic, which is the finding
                # that would matter for the paper.
                "unc_ratio_planner_over_sim": float(planned_unc / term_unc)
                if planned_unc and np.isfinite(term_unc) and term_unc > 0 else None,
                "unc_threshold_m": self.run.cfg.get("unc_radius_threshold"),
                "comm_events": len(self.comm_events),
                "landmark_events": len(self.landmark_events),
            },
        }
        for name in self.hc.get_agent_names():
            if name == p:
                continue
            a = self._arrays(name)
            out[f"support_{name}"] = {
                "max_penetration_m": float(np.max(self.pen[name])) if self.pen[name] else 0.0,
                "rms_cross_track_m": float(np.sqrt(np.mean(a["xtrack"] ** 2))) if len(a.get("xtrack", [])) else np.nan,
                "skipped_waypoints": self.skipped_waypoints.get(name, []),
            }
        return out

    def finish(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        report = self.report()
        _dump_yaml(report, os.path.join(out_dir, "sim_report.yaml"))

        blob = {
            "run_dir": np.array(self.run.run_dir), "algo": np.array(self.run.algo),
            "agents": np.array(self.hc.get_agent_names()),
            "primary": np.array(self.primary),
            "goal_xy": np.asarray(self.run.goal_xy, float),
            "landmarks": self.run.landmarks, "landmark_cov": self.run.landmark_cov,
            "agent_radius": np.array(self.agent_radius),
            "unc_threshold": np.array(self.run.cfg.get("unc_radius_threshold", np.nan), float),
            "planner_unc": np.array(self.run.meta.get("primary_unc", np.nan), float),
            "comm_events": np.array(self.comm_events, float).reshape(-1, 6),
            "landmark_events": np.array(self.landmark_events, float).reshape(-1, 5),
        }
        for i, obs in enumerate(self.run.obstacles):
            blob[f"obstacle{i}_verts"] = obs.verts
        blob["n_obstacles"] = np.array(len(self.run.obstacles))
        for name in self.hc.get_agent_names():
            blob[f"{name}_track"] = np.array(self.track[name], float).reshape(-1, 4)
            blob[f"{name}_pen"] = np.array(self.pen[name], float)
            blob[f"{name}_ref"] = self.ref[name]
            for k, v in self._arrays(name).items():
                blob[f"{name}_node_{k}"] = v
        np.savez_compressed(os.path.join(out_dir, "run_log.npz"), **blob)
        return report


def print_report(report):
    for section, body in report.items():
        print(f"\n{section}:")
        for k, v in body.items():
            print(f"  {k:<28} {v}")


def _self_check():
    """Drive a Recorder with a synthetic run whose answers are known by
    construction: a straight path that deliberately clips one obstacle."""
    import config as cfg_mod

    default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "..", "results", "paper", "montecarlo")
    run = plan_io.load_run(default)
    hc = cfg_mod.HoloOceanConfig(run)
    rec = Recorder(run, hc)

    # Fly the planned path exactly, then bend through obstacle 1 (200..260, -150..-100).
    path = np.array(hc.planned_path("auv0"))
    for i, (x, y) in enumerate(path):
        rec.tick(i / 30.0, "auv0", [x, y, -240.0])
    clip = np.array([[230.0, -125.0]])       # dead centre of obstacle 1
    rec.tick(len(path) / 30.0, "auv0", [clip[0, 0], clip[0, 1], -240.0])
    for i, (x, y) in enumerate(path[::10]):
        rec.tick(i / 30.0, "auv1", [x, y, -245.0])

    sig = np.diag([1e-4, 1e-4, 1e-4, 4.0, 9.0, 1.0])   # det(xy)^0.25 = 6^0.5
    for i, (x, y) in enumerate(path[::20]):
        rec.node("auv0", i, i * 20.0, [x, y, -240.0], [x + 1.0, y, -240.0], sig, 0.0, 0.0)
        rec.node("auv1", i, i * 20.0, [x, y, -245.0], [x, y, -245.0], sig, 0.0, 0.0)

    rep = rec.report()
    assert rep["result"]["max_penetration_m"] > 24.0, "missed a dead-centre obstacle clip"
    assert rep["result"]["first_breach_m"] is not None, "breach location not recorded"
    assert abs(rep["estimation"]["terminal_est_error_m"] - 1.0) < 1e-9, "estimate error wrong"
    assert abs(rep["estimation"]["terminal_unc_m"] - 6.0 ** 0.5) < 1e-9, \
        "unc_radius must read the [3:5,3:5] translation block of the GTSAM marginal"
    assert rep["tracking"]["max_cross_track_m"] < 1e-6, "flying the reference should be zero cross-track"

    import tempfile
    out = tempfile.mkdtemp(prefix="sim_selfcheck_")
    rec.finish(out)
    back = plan_io.read_yaml(os.path.join(out, "sim_report.yaml"))
    assert back["result"]["reached_goal"] is False, "yaml round-trip lost the verdict"
    z = np.load(os.path.join(out, "run_log.npz"), allow_pickle=False)
    assert f"auv0_node_unc" in z and z["auv0_track"].shape[1] == 4, "run_log is missing fields"
    print_report(rep)
    print(f"\nwrote {out}/  ({len(z.files)} arrays)\nOK")


if __name__ == "__main__":
    _self_check()
