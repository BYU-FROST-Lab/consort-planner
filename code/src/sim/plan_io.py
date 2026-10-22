"""Ingest a `landmark_planning` run directory.

This is the only module that knows the planner's file schemas. Everything else
in the repo takes a `Run`.

Layout produced by generate_plan.jl:

    results/<timestamp>/
        scenario_landmarks.csv          landmark_id,x,y,c11,c12,c21,c22
        scenario_obstacles.csv          obstacle_id,vertex_index,x,y,s11,s12,s21,s22
        config/main.yaml                the config the run used
        <algo>/results.yaml             run manifest
        <algo>/csv/main_ctrls.csv       agent,ctrl_index,x,y
        <algo>/csv/comm_events.csv      arc_dist,agent_a,agent_b,weight,xa,ya,xb,yb
        <algo>/csv/landmark_events.csv  arc_dist,agent,landmark_id,quality,xa,ya,xl,yl

Two facts about main_ctrls.csv that are easy to get wrong:

  * The rows are cubic B-spline CONTROL POINTS, not waypoints. Interior points
    are not on the flown curve. The knot vector is clamped, so the first and
    last points *are* interpolated -- start and goal are exact.
  * `agent` is 1-based and THE PRIMARY IS THE LAST INDEX (mc_nees.jl:353
    `const PRIMARY = NA`). Agents 1..NA-1 are supports.
"""

import csv
import os
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import BSpline

# Mirrors config/hexspline_cl.yaml: spline_degree, spline_samples_per_seg.
SPLINE_DEGREE = 3
SPLINE_SAMPLES_PER_SEG = 5


# --------------------------------------------------------------------------
# Obstacles -- a port of src/planner/obstacles.jl
# --------------------------------------------------------------------------

@dataclass
class Obstacle:
    """Convex polygon as an intersection of half-planes {x : A @ x <= b}."""
    verts: np.ndarray    # (n, 2) in polygon order
    A: np.ndarray        # (n, 2) UNIT outward face normals
    b: np.ndarray        # (n,)   face offsets
    sigma_o: np.ndarray  # (2, 2) location covariance, usually zero


def build_obstacle(verts, sigma_o=None):
    """Derive unit outward normals from ordered vertices, either winding.

    Port of build_obstacle (src/planner/obstacles.jl:64). Errors on non-convex input,
    matching the Julia side -- convex decomposition is a separate task.
    """
    verts = np.asarray(verts, dtype=float)
    n = len(verts)
    if n < 3:
        raise ValueError(f"obstacle polygon needs >=3 vertices, got {n}")
    sigma_o = np.zeros((2, 2)) if sigma_o is None else np.asarray(sigma_o, float)

    nxt = np.roll(verts, -1, axis=0)
    prv = np.roll(verts, 1, axis=0)

    s = np.sign(np.sum(verts[:, 0] * nxt[:, 1] - nxt[:, 0] * verts[:, 1]))  # winding
    if s == 0:
        raise ValueError("degenerate obstacle polygon (zero area)")

    e = nxt - verts                                    # edge j: v_j -> v_j+1
    L = np.hypot(e[:, 0], e[:, 1])
    if np.any(L <= 0):
        raise ValueError("obstacle polygon has a zero-length edge")

    A = s * np.column_stack([e[:, 1], -e[:, 0]]) / L[:, None]
    b = np.sum(A * verts, axis=1)

    cross = (verts[:, 0] - prv[:, 0]) * e[:, 1] - (verts[:, 1] - prv[:, 1]) * e[:, 0]
    if np.any(s * cross < -1e-9):
        raise ValueError("non-convex obstacle polygon")

    return Obstacle(verts=verts, A=A, b=b, sigma_o=sigma_o)


def obstacle_penetration(x, y, obs, agent_radius=0.0):
    """Depth inside the polygon in metres; 0.0 when outside.

    Port of obstacle_penetration (src/planner/minvo.jl:263), which is the Sigma=0
    geometric containment test check_true_collisions.jl scores with -- NOT the
    planner's chance-constrained feasibility test (that one operates on a
    belief, and asks a different question).

    `agent_radius` inflates every face outward, so agent_radius=0 reproduces
    the Julia number exactly and agent_radius=r_a gives hull contact for a
    vehicle of that radius. Both are worth reporting; they answer
    "where did the planner think the path went" vs "did the hull touch".
    """
    s = (obs.b + agent_radius) - (obs.A[:, 0] * x + obs.A[:, 1] * y)
    if np.any(s < 0.0):
        return 0.0
    return float(np.min(s))


def inflate(obs, r):
    """Vertices of the obstacle grown outward by `r`.

    Exact rather than a scale-from-centroid approximation: the inflated polygon
    is {x : A x <= b + r}, so each new vertex is where two consecutive offset
    faces meet. Drawing this means the boundary on screen IS the constraint
    obstacle_penetration tests against, instead of merely resembling it.
    """
    A, b = obs.A, obs.b + r
    nxt = lambda j: (j + 1) % len(b)
    verts = []
    for j in range(len(b)):
        M = np.array([A[j], A[nxt(j)]])
        if abs(np.linalg.det(M)) < 1e-12:      # parallel faces: degenerate corner
            continue
        verts.append(np.linalg.solve(M, [b[j], b[nxt(j)]]))
    return np.array(verts) if verts else obs.verts


def worst_penetration(pts, obstacles, agent_radius=0.0):
    """Deepest penetration over a path. Returns (depth, point_index, obs_index)."""
    worst, at, which = 0.0, -1, -1
    for i, (x, y) in enumerate(np.asarray(pts, float)[:, :2]):
        for m, obs in enumerate(obstacles):
            d = obstacle_penetration(x, y, obs, agent_radius)
            if d > worst:
                worst, at, which = d, i, m
    return worst, at, which


# --------------------------------------------------------------------------
# Clamped cubic B-spline -- must reproduce src/planner/algorithms/hexspline_cl.jl exactly
# --------------------------------------------------------------------------

def pad_controls(ctrl):
    """Port of bspline_pad_controls (hexspline_cl.jl:112).

    Rarely fires -- plans normally carry >=12 control points -- but omitting it
    would silently break short ones.
    """
    ctrl = np.asarray(ctrl, dtype=float)
    n = len(ctrl)
    if n >= 4:
        return ctrl
    if n == 3:
        return ctrl[[0, 0, 1, 2, 2]]
    if n == 2:
        return ctrl[[0, 0, 1, 1]]
    if n == 1:
        return ctrl[[0, 0, 0, 0]]
    return ctrl.reshape(0, 2)


def open_uniform_knots(nctrl, degree=SPLINE_DEGREE):
    """Port of bspline_open_uniform_knots (hexspline_cl.jl:127).

    The textbook clamped knot vector -- degree+1 zeros, interior knots at
    i/nspans, degree+1 ones -- which is exactly what scipy.BSpline expects, so
    no index juggling is needed beyond building the array.
    """
    if nctrl < degree + 1:
        return np.empty(0)
    nspans = max(1, nctrl - degree)
    knots = np.empty(nctrl + degree + 1)
    knots[:degree + 1] = 0.0
    knots[nctrl:] = 1.0
    knots[degree + 1:nctrl] = np.arange(1, nspans) / nspans
    return knots


def sample_spline(ctrl, degree=SPLINE_DEGREE, samples_per_seg=SPLINE_SAMPLES_PER_SEG):
    """Evaluate the planned curve. Port of bspline_sample_path (hexspline_cl.jl:217)."""
    controls = pad_controls(ctrl)
    nctrl = len(controls)
    if nctrl == 0:
        return np.empty((0, 2))
    if nctrl < degree + 1:
        return controls.copy()

    knots = open_uniform_knots(nctrl, degree)
    nspans = max(1, nctrl - degree)
    u = np.linspace(0.0, 1.0, nspans * samples_per_seg + 1)
    return BSpline(knots, controls, degree)(u)


def spline_tangent(ctrl, u=0.0, degree=SPLINE_DEGREE):
    """Unit tangent at parameter u -- used to point a vehicle along its own path."""
    controls = pad_controls(ctrl)
    if len(controls) < degree + 1:
        d = controls[-1] - controls[0]
    else:
        knots = open_uniform_knots(len(controls), degree)
        d = BSpline(knots, controls, degree).derivative()(u)
    n = np.hypot(d[0], d[1])
    return d / n if n > 1e-12 else np.array([1.0, 0.0])


# --------------------------------------------------------------------------
# Path geometry
# --------------------------------------------------------------------------

def _drop_duplicates(pts, eps=1e-9):
    """Supports are padded to the primary's control count with repeated trailing
    points (zero-length legs = idling), so a sampled support path ends in a pile
    of identical samples. Arc-length interpolation needs strict monotonicity."""
    pts = np.asarray(pts, float)
    if len(pts) < 2:
        return pts
    keep = np.ones(len(pts), dtype=bool)
    keep[1:] = np.hypot(*(pts[1:] - pts[:-1]).T) > eps
    return pts[keep]


def arclength(pts):
    """Cumulative distance along a polyline, same length as `pts`."""
    pts = np.asarray(pts, float)
    if len(pts) < 2:
        return np.zeros(len(pts))
    steps = np.hypot(*(pts[1:] - pts[:-1]).T)
    return np.concatenate([[0.0], np.cumsum(steps)])


def resample_by_arclength(pts, spacing_m):
    """Even-spaced waypoints along a densely sampled path, endpoints included.

    The dense spline samples sit ~18 m apart on a 1000 m plan, which is coarse
    against an 8 m arrival radius; this puts them where the follower wants them.
    """
    pts = _drop_duplicates(pts)
    if len(pts) < 2:
        return pts
    s = arclength(pts)
    targets = np.arange(0.0, s[-1], spacing_m)
    targets = np.append(targets, s[-1])          # always finish on the goal
    return np.column_stack([np.interp(targets, s, pts[:, 0]),
                            np.interp(targets, s, pts[:, 1])])


def cross_track_error(pos_xy, ref_pts):
    """Perpendicular distance from a point to a reference polyline."""
    p = np.asarray(pos_xy, float)[:2]
    ref = np.asarray(ref_pts, float)
    if len(ref) < 2:
        return float(np.linalg.norm(p - ref[0])) if len(ref) else np.inf
    a, b = ref[:-1], ref[1:]
    ab = b - a
    denom = np.sum(ab * ab, axis=1)
    t = np.clip(np.sum((p - a) * ab, axis=1) / np.where(denom > 0, denom, 1.0), 0.0, 1.0)
    closest = a + t[:, None] * ab
    return float(np.min(np.hypot(*(p - closest).T)))


# --------------------------------------------------------------------------
# Readers
# --------------------------------------------------------------------------

def _rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _scalar(tok):
    tok = tok.strip()
    if tok.startswith("[") and tok.endswith("]"):
        inner = tok[1:-1].strip()
        return [_scalar(t) for t in inner.split(",")] if inner else []
    if tok in ("true", "false"):
        return tok == "true"
    try:
        return int(tok)
    except ValueError:
        pass
    try:
        return float(tok)
    except ValueError:
        return tok


def read_yaml(path):
    """Minimal reader for the flat `key: value` YAML this pipeline writes.

    Handles one level of nesting (results.yaml's `main:` block) and inline
    lists. Deliberately not PyYAML: adding a dependency to parse two files this
    simple is not worth it, and the planner's own loader (src/planner/config.jl) is
    just as restrictive about what it will accept.
    """
    out, section = {}, None
    with open(path) as f:
        for raw in f:
            line = raw.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            indented = line[0] in " \t"
            key, sep, val = line.strip().partition(":")
            if not sep:
                continue
            key, val = key.strip(), val.strip()
            if not val:
                section = out.setdefault(key, {})
                continue
            (section if indented and section is not None else out)[key] = _scalar(val)
    return out


def _load_obstacles(path):
    if not os.path.exists(path):
        return []
    by_id = {}
    for r in _rows(path):
        by_id.setdefault(int(r["obstacle_id"]), []).append(
            (int(r["vertex_index"]), float(r["x"]), float(r["y"]),
             [[float(r["s11"]), float(r["s12"])], [float(r["s21"]), float(r["s22"])]]))
    obstacles = []
    for oid in sorted(by_id):
        rows = sorted(by_id[oid], key=lambda t: t[0])
        obstacles.append(build_obstacle([(x, y) for _, x, y, _ in rows], rows[0][3]))
    return obstacles


# --------------------------------------------------------------------------
# Run
# --------------------------------------------------------------------------

@dataclass
class Run:
    """One planner run, fully self-describing."""
    run_dir: str
    algo: str
    ctrls: list          # ctrls[i] = (n_i, 2) control points for planner agent i+1
    landmarks: np.ndarray        # (M, 2)
    landmark_cov: np.ndarray     # (M, 2, 2)
    obstacles: list
    cfg: dict            # config/main.yaml
    meta: dict           # <algo>/results.yaml, `main` block
    comm_events: list
    landmark_events: list

    @property
    def num_agents(self):
        return len(self.ctrls)

    @property
    def primary_idx(self):
        """Planner convention: the primary is the LAST agent index."""
        return self.num_agents - 1

    @property
    def goal_xy(self):
        return self.ctrls[self.primary_idx][-1]

    @property
    def sigma0(self):
        """Initial agent covariance. The planner stores it as landmark 1's
        covariance (src/planner/viz.jl:241) -- a quirk, but it is the documented pin."""
        return self.landmark_cov[0] if len(self.landmark_cov) else None

    def path(self, i, samples_per_seg=SPLINE_SAMPLES_PER_SEG):
        """Densely sampled planned curve for agent i (0-based)."""
        return sample_spline(self.ctrls[i], samples_per_seg=samples_per_seg)

    def waypoints(self, i, spacing_m):
        return resample_by_arclength(self.path(i), spacing_m)


def load_run(run_dir, algo=None, require_optimized=True):
    """Read a planner run directory into a `Run`.

    `require_optimized` rejects plans whose continuous refinement failed. Flying
    an uncertified path would look exactly like a controller bug at the other
    end, so it is worth failing loudly here.
    """
    run_dir = os.path.abspath(run_dir)
    if algo is None:
        cands = sorted(d for d in os.listdir(run_dir)
                       if os.path.isfile(os.path.join(run_dir, d, "results.yaml")))
        if len(cands) != 1:
            raise ValueError(f"pass algo= explicitly; found {cands or 'no planners'} in {run_dir}")
        algo = cands[0]

    csv_dir = os.path.join(run_dir, algo, "csv")
    meta = read_yaml(os.path.join(run_dir, algo, "results.yaml")).get("main", {})
    status = meta.get("refinement_status")
    if require_optimized and status not in ("optimized", None):
        raise ValueError(
            f"{run_dir}/{algo}: refinement_status={status!r} -- the plan violates its own "
            f"constraints. Pass require_optimized=False to fly it anyway.")

    by_agent = {}
    for r in _rows(os.path.join(csv_dir, "main_ctrls.csv")):
        by_agent.setdefault(int(r["agent"]), []).append(
            (int(r["ctrl_index"]), float(r["x"]), float(r["y"])))
    if sorted(by_agent) != list(range(1, len(by_agent) + 1)):
        raise ValueError(f"main_ctrls.csv must have rows for agents 1..N, got {sorted(by_agent)}")
    ctrls = [np.array([[x, y] for _, x, y in sorted(by_agent[a], key=lambda t: t[0])])
             for a in sorted(by_agent)]

    lm_rows = _rows(os.path.join(run_dir, "scenario_landmarks.csv"))
    landmarks = np.array([[float(r["x"]), float(r["y"])] for r in lm_rows]).reshape(-1, 2)
    landmark_cov = np.array([[[float(r["c11"]), float(r["c12"])],
                              [float(r["c21"]), float(r["c22"])]] for r in lm_rows]).reshape(-1, 2, 2)

    def _events(name):
        p = os.path.join(csv_dir, name)
        return _rows(p) if os.path.exists(p) else []

    return Run(
        run_dir=run_dir,
        algo=algo,
        ctrls=ctrls,
        landmarks=landmarks,
        landmark_cov=landmark_cov,
        obstacles=_load_obstacles(os.path.join(run_dir, "scenario_obstacles.csv")),
        cfg=read_yaml(os.path.join(run_dir, "config", "main.yaml")),
        meta=meta,
        comm_events=_events("comm_events.csv"),
        landmark_events=_events("landmark_events.csv"),
    )


# --------------------------------------------------------------------------

def _self_check(run_dir):
    """Cross-check the Python spline against the planner's own numbers.

    Arc length is the single assertion that exercises padding, the knot vector
    and the sampling all at once: if any of the three is wrong, it drifts.
    """
    run = load_run(run_dir)
    print(f"run    : {run.run_dir} [{run.algo}]")
    print(f"agents : {run.num_agents}  primary=agent {run.primary_idx + 1}")

    pts = run.path(run.primary_idx)
    start, goal = run.ctrls[run.primary_idx][0], run.goal_xy

    # Clamped knots interpolate the endpoints exactly.
    assert np.allclose(pts[0], start, atol=1e-9), f"start {pts[0]} != {start}"
    assert np.allclose(pts[-1], goal, atol=1e-9), f"goal {pts[-1]} != {goal}"

    got = arclength(pts)[-1]
    want = run.meta.get("continuous_primary_length") or run.meta.get("primary_length")
    err = abs(got - want) / want
    print(f"length : {got:.3f} m vs results.yaml {want} ({err * 100:.3f}% off)")
    assert err < 5e-3, f"arc length off by {err * 100:.2f}% -- knots or padding are wrong"

    # Obstacle round-trip: a vertex sits on the boundary, the centroid inside.
    for i, obs in enumerate(run.obstacles):
        assert obstacle_penetration(*obs.verts[0], obs) == 0.0, f"obstacle {i}: vertex reads inside"
        c = obs.verts.mean(axis=0)
        assert obstacle_penetration(*c, obs) > 0.0, f"obstacle {i}: centroid reads outside"
    print(f"obstacles: {len(run.obstacles)} ok   landmarks: {len(run.landmarks)}")

    wps = run.waypoints(run.primary_idx, 25.0)
    assert np.allclose(wps[-1], goal, atol=1e-6), "resampling lost the goal"
    assert np.all(np.hypot(*(wps[1:] - wps[:-1]).T) <= 25.0 + 1e-6), "waypoint spacing exceeded"
    print(f"waypoints: {len(wps)} at 25 m spacing, ending on the goal")

    worst, at, which = worst_penetration(pts, run.obstacles,
                                         run.cfg.get("agent_radius", 0.0))
    print(f"planned path vs obstacles (r_a={run.cfg.get('agent_radius')}): "
          f"worst={worst:.3f} m" + (f" at sample {at}, obstacle {which}" if worst else ""))
    print("\nOK")


if __name__ == "__main__":
    import sys

    default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "..", "results", "paper", "montecarlo")
    _self_check(sys.argv[1] if len(sys.argv) > 1 else default)
