"""Three views of a run, one visual language.

    SimOverlay   3D, in the HoloOcean viewport, via env.draw_*  -- the demo
    LiveView     2D matplotlib, throttled, persistent artists   -- the instrument
    figures      2D matplotlib -> PDF+PNG, offline from run_log.npz -- the paper

The split is by timing, not importance. The live views are drawn incrementally
while the run happens; the figures are drawn afterwards from the recorded log
and never touch the simulator. That is why `python viz.py <sim_dir>` works with
nothing else running, and why re-styling a figure costs no simulation time.

Style is deliberately copied from the planner's own plots (landmark_planning
src/planner/viz.jl), because figures from both repos land in the same paper. The colour
table, the 2-sigma landmark ellipses and their display_scale=400 inflation are
matched there; if that file's style changes, mirror it here.

One thing NOT matched, on purpose: display_scale inflates LANDMARK covariance
ellipses only. Agent pose covariance is drawn at true 2-sigma, unscaled --
those are the numbers under test, and inflating them would be a lie.
"""

import os

import numpy as np

import plan_io

# ---------------------------------------------------------------------------
# Style -- mirrors landmark_planning/src/viz.jl
# ---------------------------------------------------------------------------

PRIMARY_COLOR = "#0000FF"                       # :blue
SUPPORT_COLORS = ["#800080", "#008080", "#FF8C00", "#DC143C", "#FF00FF",
                  "#A52A2A", "#00FF00", "#000080", "#FF7F50", "#808000"]
OBSTACLE_FILL = "#595959"                       # :gray35
LANDMARK_COLOR = "black"
LANDMARK_ELLIPSE = "red"
START_COLOR = "green"
GOAL_COLOR = "orange"
TRUTH_COLOR = "#1a1a1a"
EST_COLOR = "#e41a1c"
BREACH_COLOR = "red"

LANDMARK_DISPLAY_SCALE = 400.0                  # viz.jl:66 -- viz only, not estimation
LANDMARK_NSTD = 2
EVENT_FADE_FRAMES = 30                          # ~1s of SIM TIME -- how long an event marker glows
# Waypoint markers lit ahead of each vehicle at any moment (None = the whole
# list). The route is already drawn as a persistent line by draw_static, so the
# markers only need to say "these are the next ones", not restate the path.
WAYPOINT_LOOKAHEAD = 3
# The map/uncertainty panels are re-rendered by matplotlib once per frame, at
# ~100 ms each -- that is the whole cost of a video, not the simulation (a
# 20k-tick mission is ~11 min to fly and would be ~57 min to render at 1:1).
# Rendering every VIDEO_STRIDE-th tick and encoding at 30/VIDEO_STRIDE fps
# keeps playback the same real-time speed for a third of the wall clock; the
# 3D close-ups still come from the sim at the full tick rate, and the composite
# in record_video.py normalises every input back to 30 fps so they stay in sync.
VIDEO_STRIDE = 3

RC = {
    "pdf.fonttype": 42, "ps.fonttype": 42,      # embed real fonts; IEEE rejects Type-3
    "font.family": "serif", "font.size": 9,
    "axes.titlesize": 10, "axes.labelsize": 9,
    "legend.fontsize": 7.5, "legend.frameon": False,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linewidth": 0.5,
    "lines.linewidth": 1.2, "figure.constrained_layout.use": True,
    "savefig.bbox": "tight", "savefig.dpi": 300,
}


def use_style():
    import matplotlib
    matplotlib.rcParams.update(RC)


def agent_color(name, primary):
    if name == primary:
        return PRIMARY_COLOR
    i = int(name[3:]) if name[3:].isdigit() else 0
    return SUPPORT_COLORS[(i - 1) % len(SUPPORT_COLORS)]


def cov_ellipse_pts(mu, cov, nstd=2, scale=1.0, n=64):
    """Points of an n-sigma covariance ellipse. `scale` is display inflation."""
    cov = np.asarray(cov, float)[:2, :2] * scale
    vals, vecs = np.linalg.eigh((cov + cov.T) / 2)
    vals = np.clip(vals, 0, None)
    th = np.linspace(0, 2 * np.pi, n)
    pts = (vecs @ np.diag(nstd * np.sqrt(vals))) @ np.vstack([np.cos(th), np.sin(th)])
    return np.asarray(mu, float)[:2, None] + pts


# ---------------------------------------------------------------------------
# Scene primitives -- shared by the live panel and the figures
# ---------------------------------------------------------------------------

def draw_obstacles(ax, obstacles, agent_radius=0.0, label=True):
    """Filled polygons plus the inflated boundary the planner actually enforced.

    Showing only the raw polygon would misrepresent a path that hugs the
    inflated edge: that path is obeying its constraint, not misbehaving.
    """
    from matplotlib.patches import Polygon
    for k, obs in enumerate(obstacles):
        ax.add_patch(Polygon(obs.verts, closed=True, facecolor=OBSTACLE_FILL,
                             alpha=0.55, edgecolor="black", linewidth=1.5, zorder=2,
                             label="Obstacle" if (label and k == 0) else None))
        if agent_radius > 0:
            infl = plan_io.inflate(obs, agent_radius)
            ax.add_patch(Polygon(infl, closed=True, fill=False, edgecolor="black",
                                 linestyle="--", linewidth=0.8, alpha=0.6, zorder=2,
                                 label=f"+{agent_radius:g} m agent radius" if (label and k == 0) else None))


def draw_landmarks(ax, landmarks, cov=None, detected=None, label=True):
    """Black dots + red 2-sigma ellipses. `detected` fills the ones the sim
    actually observed and leaves the rest hollow -- the planner predicts
    detections from a range model, the sim observes them through a moving,
    yawing vehicle, and the disagreement is a finding."""
    if not len(landmarks):
        return
    landmarks = np.asarray(landmarks, float)
    if cov is not None:
        for mu, c in zip(landmarks, cov):
            e = cov_ellipse_pts(mu, c, LANDMARK_NSTD, LANDMARK_DISPLAY_SCALE)
            ax.fill(e[0], e[1], color=LANDMARK_ELLIPSE, alpha=0.25, zorder=1,
                    label=f"Landmark {LANDMARK_NSTD}$\\sigma$ ($\\times${LANDMARK_DISPLAY_SCALE:g})"
                    if (label and mu is landmarks[0]) else None)
    if detected is None:
        ax.plot(landmarks[:, 0], landmarks[:, 1], ".", color=LANDMARK_COLOR,
                markersize=4, linestyle="none", zorder=4,
                label="Landmarks" if label else None)
    else:
        d = np.asarray(detected, bool)
        ax.plot(landmarks[d, 0], landmarks[d, 1], "o", color=LANDMARK_COLOR, markersize=4,
                linestyle="none", zorder=4, label="Landmark (detected)" if label else None)
        ax.plot(landmarks[~d, 0], landmarks[~d, 1], "o", markerfacecolor="none",
                markeredgecolor=LANDMARK_COLOR, markersize=4, linestyle="none", zorder=4,
                label="Landmark (never seen)" if label else None)


def draw_planned_path(ax, ref, color="0.45", label=None):
    ax.plot(ref[:, 0], ref[:, 1], "--", color=color, linewidth=1.0, zorder=3, label=label)


def draw_start_goal(ax, start, goal, label=True):
    ax.plot(*start[:2], "o", color=START_COLOR, markersize=8, zorder=6,
            label="Start" if label else None)
    ax.plot(*goal[:2], "*", color=GOAL_COLOR, markersize=13, zorder=6,
            label="Goal" if label else None)


def draw_breaches(ax, track_xy, pen, label=True):
    """Overdraw any part of the flown path inside an obstacle. This is the
    pass/fail signal; it should be impossible to miss."""
    bad = np.asarray(pen) > 0
    if not bad.any():
        return 0.0
    ax.plot(np.where(bad, track_xy[:, 0], np.nan),
            np.where(bad, track_xy[:, 1], np.nan),
            color=BREACH_COLOR, linewidth=3.5, zorder=7, solid_capstyle="butt",
            label="Obstacle breach" if label else None)
    i = int(np.argmax(pen))
    ax.annotate(f"breach {np.max(pen):.1f} m", xy=track_xy[i], xytext=(12, 12),
                textcoords="offset points", color=BREACH_COLOR, fontsize=8,
                arrowprops=dict(arrowstyle="->", color=BREACH_COLOR, lw=0.8))
    return float(np.max(pen))


# ---------------------------------------------------------------------------
# The 3D in-sim overlay -- the demo view
# ---------------------------------------------------------------------------

def _rgb(hex_or_list):
    if isinstance(hex_or_list, (list, tuple)):
        return list(hex_or_list)
    h = hex_or_list.lstrip("#")
    return [int(h[i:i + 2], 16) for i in (0, 2, 4)]


class SimOverlay:
    """Draws the scene into the HoloOcean viewport.

    Passive by construction: it only issues draw commands, never reads state
    back into anything that is scored. With `enabled=False` every method is a
    no-op, so a headless run must produce an identical log.
    """

    def __init__(self, env, hc, run, enabled=True, chase=True):
        self.env, self.hc, self.run = env, hc, run
        self.enabled, self.chase = enabled, chase
        self.primary = hc.cfg["main_agent"]
        self._dash_queue = []  # see flush_dashes()

    def _line(self, a, b, color, thickness=4, lifetime=0.0):
        self.env.draw_line([float(v) for v in a], [float(v) for v in b],
                           color=_rgb(color), thickness=thickness, lifetime=lifetime)

    # Dash / gap length, in metres of actual arc length along the planned
    # path (not sample count -- the B-spline's control points are not evenly
    # spaced, so at 0.1 m a sample-index-based dash would run short or long
    # depending where a segment fell; this instead walks real cumulative
    # distance, so every dash and gap comes out the same length everywhere).
    DASH_ON_M = 0.5
    DASH_OFF_M = 0.5
    # Sample spacing must stay well under DASH_ON_M/DASH_OFF_M or a dash or
    # gap can be skipped entirely between two samples. 5000 samples per
    # B-spline segment measures out to <=0.086 m worst-case spacing on this
    # plan's ~1082 m path (~50k samples) -- dense enough. Consecutive "on"
    # samples are merged into one draw_line per dash before queuing (see
    # draw_static()/flush_dashes()), not issued one call per sample.
    DASH_SAMPLES_PER_SEG = 5000
    # holoocean's command buffer is a fixed 1 MiB flushed on the next tick();
    # a DebugDraw command serializes to ~254 bytes, so ~4100 of them is the
    # hard ceiling for everything queued in a single tick, and this plan's
    # ~0.1 m dashes alone need ~10-11k calls across both agents. Queue them
    # in draw_static() and drain a safe batch per tick via flush_dashes() --
    # measured crash otherwise: "Command length exceeds buffer size".
    DASH_FLUSH_PER_TICK = 2000

    def draw_static(self):
        """Obstacles, inflated rings and every planned path -- drawn once,
        persistent (lifetime=0), the planned path DASHED. The solid
        counterpart -- the actual flown track -- is drawn live, tick by
        tick, by `flown()` below; the two together are the whole story in
        one image: here is the line someone else planned, here is the line
        the vehicle is actually flying."""
        if not self.enabled:
            return
        z = -self.hc.depth(self.primary)
        for obs in self.run.obstacles:
            for verts, col, th in ((obs.verts, [220, 40, 40], 8),
                                   (plan_io.inflate(obs, float(self.run.cfg.get("agent_radius", 0.0))),
                                    [140, 140, 140], 3)):
                ring = np.vstack([verts, verts[:1]])
                for a, b in zip(ring[:-1], ring[1:]):
                    self._line([a[0], a[1], z], [b[0], b[1], z], col, th)
        for name in self.hc.get_agent_names():
            col = [90, 90, 255] if name == self.primary else [160, 120, 200]
            # Denser than the global default (5/seg) and undecimated -- the
            # global default is thin enough already, and this path is drawn
            # as chunky Unreal debug-line segments with no antialiasing, so
            # it needs more points than matplotlib's smooth render to read as
            # a curve rather than a facsimile of one.
            ref, zn = self.hc.planned_path(name, samples_per_seg=self.DASH_SAMPLES_PER_SEG), -self.hc.depth(name)
            seg_len = np.linalg.norm(np.diff(ref, axis=0), axis=1)
            arc = np.concatenate([[0.0], np.cumsum(seg_len)])
            cycle = self.DASH_ON_M + self.DASH_OFF_M
            on = (arc[:-1] % cycle) < self.DASH_ON_M
            # Merge consecutive "on" samples into ONE draw_line per dash --
            # one call per raw sample (at this sampling density, several per
            # dash) is what overflowed the command buffer in the first place.
            change = np.flatnonzero(np.diff(on.astype(np.int8)) != 0) + 1
            starts = np.concatenate([[0], change])
            ends = np.concatenate([change, [len(on)]])
            for s, e in zip(starts, ends):
                if on[s]:
                    a, b = ref[s], ref[e]
                    self._dash_queue.append(([a[0], a[1], zn], [b[0], b[1], zn], col))

    def flush_dashes(self):
        """Drain a safe-sized batch of queued planned-path dashes (see
        DASH_FLUSH_PER_TICK). Call once per tick from main's loop; a no-op
        once the queue is empty, which is most of every flight."""
        if not self.enabled or not self._dash_queue:
            return
        for _ in range(min(self.DASH_FLUSH_PER_TICK, len(self._dash_queue))):
            a, b, col = self._dash_queue.pop()
            self._line(a, b, col, 3)

    def flown(self, name, prev_xy, xy):
        """The actual track, solid, one segment longer every tick.

        Same per-agent color as the dashed planned path in draw_static() --
        color says which agent, dashed-vs-solid says intended-vs-actual.
        Persistent (lifetime=0), same as the planned path, so it accumulates
        behind the vehicle instead of fading like the heading arrow does.
        """
        if not self.enabled:
            return
        col = [90, 90, 255] if name == self.primary else [160, 120, 200]
        z = -self.hc.depth(name)
        self._line([prev_xy[0], prev_xy[1], z], [xy[0], xy[1], z], col, 4)

    def draw_waypoints(self, name, waypoints, start=0):
        """Light only the next WAYPOINT_LOOKAHEAD markers, not all ~50 at once.

        Drawn with a lifetime just longer than one tick and re-issued every
        tick from main's loop, so the lit window slides with the vehicle.
        Drawing the whole list persistently (lifetime=0, as this did) puts a
        solid magenta band across the horizon for the entire mission -- every
        marker sits at the agent's own depth, so from a chase view they stack
        up edge-on into one bar.
        """
        if not self.enabled:
            return
        z = -self.hc.depth(name)
        end = len(waypoints) if WAYPOINT_LOOKAHEAD is None else start + WAYPOINT_LOOKAHEAD
        for wp in waypoints[start:end]:
            self.env.draw_point(loc=[float(wp[0]), float(wp[1]), z],
                                color=[255, 0, 255], thickness=18,
                                # SIM seconds: it only has to outlive one tick.
                                lifetime=3.0 / self.hc.ticks_per_sec)

    def waypoint_reached(self, name, wp):
        if not self.enabled:
            return
        self.env.draw_point(loc=[float(wp[0]), float(wp[1]), -self.hc.depth(name)],
                            color=[0, 255, 0], thickness=18, lifetime=0)

    def heading(self, name, pos, heading_deg, on_depth=True):
        if not self.enabled:
            return
        r = np.radians(heading_deg)
        self.env.draw_arrow(
            [float(v) for v in pos],
            end=[float(pos[0] + 8 * np.cos(r)), float(pos[1] + 8 * np.sin(r)), float(pos[2])],
            color=[0, 255, 0] if on_depth else [255, 0, 0], thickness=5, lifetime=0.2)

    def estimate(self, pos_true, pos_est):
        """The money visual: a marker at the factor graph's belief and a line to
        where the vehicle really is. You watch it drift between fixes and snap
        back on a landmark detection or a comms event."""
        if not self.enabled:
            return
        self.env.draw_point(loc=[float(v) for v in pos_est], color=[255, 60, 60],
                            thickness=12, lifetime=0)
        self._line(pos_true, pos_est, [255, 60, 60], 3, lifetime=0.0)

    def comm(self, a_pos, b_pos):
        if self.enabled:
            self._line(a_pos, b_pos, [255, 220, 0], 5, lifetime=1.0)

    def landmark_ray(self, agent_pos, landmark_xy, z):
        if self.enabled:
            self._line(agent_pos, [landmark_xy[0], landmark_xy[1], z], [0, 200, 255], 2, lifetime=0.5)

    def follow(self, pos, heading_deg):
        """Chase camera. Without it the camera stares at empty water while the
        vehicle drives a kilometre out of frame."""
        if not (self.enabled and self.chase):
            return
        r = np.radians(heading_deg)
        # move_viewport's yaw is in Unreal's left-handed camera convention,
        # opposite sign from heading_deg (world frame, matches DynamicsSensorRPY
        # and the position offset below) -- passing heading_deg straight through
        # points the camera off into open water instead of at the vehicle.
        # Distance/height/pitch kept modest: this world's kelp is dense enough
        # that the original 28m/12m/-18deg framing buried the vehicle in
        # foliage and rendered enough canopy to visibly tank the framerate.
        # NOTE the sign convention here is NOT the one a mounted camera sensor
        # uses -- measured on an RGBCamera, POSITIVE pitch looks down (see
        # config.CLOSEUP_CAM_ROT). These numbers were never re-checked against that,
        # and this whole path is off by default (main.CHASE_CAM), so don't copy
        # them onto a sensor.
        dist, height, pitch = 12.0, 4.0, -10.0
        self.env.move_viewport(
            [float(pos[0] - dist * np.cos(r)), float(pos[1] - dist * np.sin(r)), float(pos[2] + height)],
            [0.0, pitch, -float(heading_deg)])


# ---------------------------------------------------------------------------
# The 2D live panel -- the instrument
# ---------------------------------------------------------------------------

class LiveView:
    """Plan view + an uncertainty strip, updated at the factor-graph cadence.

    Persistent artists, not clf()-and-replot: obstacles and landmarks are static
    and get drawn exactly once, and the growing trajectories are updated with
    set_data. The old code cleared and replotted everything every tick, which
    grows without bound over a 1000 m path and ends up costing more than the
    physics.
    """

    def __init__(self, hc, run, enabled=True):
        self.enabled = enabled
        if not enabled:
            return
        import matplotlib
        import matplotlib.pyplot as plt
        use_style()
        self.plt = plt
        self.hc, self.run = hc, run
        self.primary = hc.cfg["main_agent"]

        self.fig, (self.ax, self.axu) = plt.subplots(
            2, 1, figsize=(11, 8), height_ratios=[3, 1], num="sim")
        r = float(run.cfg.get("agent_radius", 0.0))
        draw_obstacles(self.ax, run.obstacles, r)
        draw_landmarks(self.ax, run.landmarks, run.landmark_cov)
        for name in hc.get_agent_names():
            ref = hc.planned_path(name)
            draw_planned_path(self.ax, ref, label="planned" if name == self.primary else None)
        draw_start_goal(self.ax, run.ctrls[run.primary_idx][0], run.goal_xy)

        self.truth, self.est, self.ell, self.unc = {}, {}, {}, {}
        for name in hc.get_agent_names():
            c = agent_color(name, self.primary)
            self.truth[name], = self.ax.plot([], [], "-", color=c, lw=1.6,
                                             label=f"{name} flown", zorder=5)
            self.est[name], = self.ax.plot([], [], ":", color=c, lw=1.2, alpha=0.85,
                                           label=f"{name} estimate", zorder=5)
            self.ell[name], = self.ax.plot([], [], "-", color=c, lw=0.8, alpha=0.6, zorder=5)
            self.unc[name], = self.axu.plot([], [], "-", color=c, lw=1.4, label=name)
        self.comm_art, = self.ax.plot([], [], "-", color="0.3", lw=0.7, alpha=0.7, zorder=4)

        self.ax.set_xlabel("x (m)"); self.ax.set_ylabel("y (m)")
        self.ax.set_aspect("equal", adjustable="datalim")
        self.ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5))
        thr = run.cfg.get("unc_radius_threshold")
        if thr:
            self.axu.axhline(thr, color="red", ls=":", lw=1.5, label="threshold")
        self.axu.set_xlabel("distance travelled (m)")
        self.axu.set_ylabel(r"det$(\Sigma)^{1/4}$ (m)")
        self.axu.legend(loc="upper left", ncol=3)

        # Limits from the plan's own extents. The old code hardcoded
        # xlim(-10, 90) for an 80 m sinusoid -- wrong by ~12x here.
        allpts = np.vstack([hc.planned_path(n) for n in hc.get_agent_names()]
                           + ([run.landmarks] if len(run.landmarks) else [])
                           + [o.verts for o in run.obstacles])
        pad = 60.0
        self.ax.set_xlim(allpts[:, 0].min() - pad, allpts[:, 0].max() + pad)
        self.ax.set_ylim(allpts[:, 1].min() - pad, allpts[:, 1].max() + pad)
        # Only a GUI backend can show a window; under Agg (headless, or a
        # figure-only smoke test) fig.show() just warns on every construction.
        if matplotlib.get_backend().lower() not in ("agg", "pdf", "ps", "svg", "template"):
            plt.ion()
            self.fig.show()

    def update(self, rec, comm_segments=()):
        if not self.enabled:
            return
        for name in self.hc.get_agent_names():
            tr = np.array(rec.track[name], float).reshape(-1, 4)
            if len(tr):
                self.truth[name].set_data(tr[:, 1], tr[:, 2])
            nodes = rec._arrays(name)
            if len(nodes.get("est", [])):
                self.est[name].set_data(nodes["est"][:, 0], nodes["est"][:, 1])
                self.unc[name].set_data(nodes["arc"], nodes["unc"])
                e = cov_ellipse_pts(nodes["est"][-1], nodes["sigma"][-1][3:5, 3:5], nstd=2)
                self.ell[name].set_data(e[0], e[1])
        if comm_segments:
            xs, ys = [], []
            for (a, b) in comm_segments:
                xs += [a[0], b[0], np.nan]; ys += [a[1], b[1], np.nan]
            self.comm_art.set_data(xs, ys)
        self.axu.relim(); self.axu.autoscale_view()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def pause(self, s=0.001):
        if self.enabled:
            self.plt.pause(s)

    def close(self):
        if self.enabled:
            self.plt.close(self.fig)


# ---------------------------------------------------------------------------
# Paper figures -- offline, from run_log.npz only
# ---------------------------------------------------------------------------

class Log:
    """Thin reader for run_log.npz so the figures stay readable."""

    def __init__(self, path):
        if os.path.isdir(path):
            path = os.path.join(path, "run_log.npz")
        self.z = np.load(path, allow_pickle=False)
        self.dir = os.path.dirname(os.path.abspath(path))
        self.agents = [str(a) for a in self.z["agents"]]
        self.primary = str(self.z["primary"])

    def __getitem__(self, k):
        return self.z[k]

    def has(self, k):
        return k in self.z.files

    def node(self, name, field):
        k = f"{name}_node_{field}"
        return self.z[k] if k in self.z.files else np.empty(0)

    @property
    def obstacles(self):
        return [plan_io.build_obstacle(self.z[f"obstacle{i}_verts"])
                for i in range(int(self.z["n_obstacles"]))]


def _finish(fig, out_dir, stem):
    import matplotlib.pyplot as plt
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    for ext in ("pdf", "png"):
        p = os.path.join(out_dir, f"{stem}.{ext}")
        fig.savefig(p)
        paths.append(p)
    plt.close(fig)
    return paths


def render_map_video(sim_dir, out_dir, progress_every=500, stride=VIDEO_STRIDE):
    """One 2D top-down PNG per tick, from a completed run's `run_log.npz`.

    This is the map-view half of `record_video.py`'s synced video. HoloOcean's
    own 3D viewport can't render a usable top-down shot in this build --
    camera pitch steeper than about -25 degrees renders as an empty view
    regardless of position (confirmed on both the runtime `move_viewport`
    teleport and a statically-configured camera sensor), so a real overhead
    render isn't available. `{name}_track` is recorded every tick
    (`scoring.Recorder.tick`, called unconditionally each loop iteration), the
    same cadence RECORD writes 3D frames at, so frame `i` here and frame `i`
    of the close-up video are the same simulated instant with no timestamp
    matching needed.

    Beyond the growing truth track, this also animates: comm/landmark event
    flashes (fading over `EVENT_FADE_FRAMES`), each agent's true 2-sigma pose
    ellipse (grows/shrinks as new factor-graph nodes land -- a support's will
    grow and never shrink in XY, which is the "along-track position is
    unobservable" modelling gap, not a bug), and each agent's FG estimate as
    a dotted line alongside its solid truth track. Ellipses/estimate are held
    constant between an agent's own node ticks, same as reality: nothing new
    is known until the next node.
    """
    import matplotlib
    matplotlib.use("Agg", force=False)
    import matplotlib.pyplot as plt
    use_style()
    # Video frames need an EXACT, even-numbered pixel size (libx264 requires
    # even width/height, and record_video.py's hstack needs an exact match
    # with the 3D capture's 1280x720) -- the house style's
    # savefig.bbox="tight" and savefig.dpi=300 both fight that (tight
    # autocrops per-frame to content bounds; passing dpi=None/bbox_inches=None
    # to savefig falls BACK to those same rcParams instead of overriding
    # them), so both are pinned back to their non-house defaults here.
    matplotlib.rcParams["savefig.bbox"] = None
    matplotlib.rcParams["savefig.dpi"] = "figure"
    log = Log(sim_dir)
    os.makedirs(out_dir, exist_ok=True)

    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100)
    fig.set_layout_engine(None)  # fixed axes rect below instead
    ax.set_position([0.08, 0.08, 0.9, 0.86])
    r = float(log["agent_radius"])
    draw_obstacles(ax, log.obstacles, r)
    draw_landmarks(ax, log["landmarks"], log["landmark_cov"])

    tracks, lines = {}, {}
    est_lines, ellipses = {}, {}
    node_ticks, node_est, node_sigma, node_idx = {}, {}, {}, {}
    for name in log.agents:
        c = agent_color(name, log.primary)
        draw_planned_path(ax, log[f"{name}_ref"], label="planned" if name == log.primary else None)
        tracks[name] = log[f"{name}_track"]  # (N, 4): t, x, y, z -- one row per tick
        lines[name], = ax.plot([], [], "-", color=c, lw=1.8, zorder=5,
                               label=f"{name} flown" + (" (primary)" if name == log.primary else ""))

        node_ticks[name] = log.node(name, "tick").astype(int)
        node_est[name] = log.node(name, "est")
        node_sigma[name] = log.node(name, "sigma")
        node_idx[name] = -1
        est_lines[name], = ax.plot([], [], ":", color=c, lw=1.2, alpha=0.9, zorder=5,
                                   label=f"{name} FG estimate")
        poly, = ax.fill([0, 0], [0, 0], color=c, alpha=0.15, edgecolor=c,
                        linewidth=0.8, zorder=3)
        poly.set_visible(False)
        ellipses[name] = poly

    # Comm/landmark event markers -- one artist each, created invisible and
    # faded in/out around their recorded tick rather than accumulated, so a
    # long mission's dozens of comm events don't clutter the map.
    event_markers = []
    for e in log["comm_events"]:
        sx, sy, rx, ry, ev_tick = e[1], e[2], e[3], e[4], e[5]
        art, = ax.plot([sx, rx], [sy, ry], "-", color="0.25", lw=1.5, alpha=0, zorder=4)
        event_markers.append((int(ev_tick), art))
    for e in log["landmark_events"]:
        lx, ly, ev_tick = e[2], e[3], e[4]
        art, = ax.plot([lx], [ly], "o", markerfacecolor="none", markeredgecolor=LANDMARK_ELLIPSE,
                       markeredgewidth=2, markersize=10, alpha=0, zorder=6)
        event_markers.append((int(ev_tick), art))

    allpts = np.vstack([tracks[n][:, 1:3] for n in log.agents if len(tracks[n])]
                       + ([log["landmarks"]] if len(log["landmarks"]) else [])
                       + [o.verts for o in log.obstacles])
    pad = 60.0
    ax.set_xlim(allpts[:, 0].min() - pad, allpts[:, 0].max() + pad)
    ax.set_ylim(allpts[:, 1].min() - pad, allpts[:, 1].max() + pad)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title("Mission map")
    ax.legend(loc="upper left", fontsize=7)

    n_ticks = max((len(t) for t in tracks.values()), default=0)
    # `i` indexes SIM TICKS (so event fades and node lookups stay in sim time);
    # `frame` numbers the PNGs contiguously, which is what ffmpeg's %06d needs.
    for frame, i in enumerate(range(0, n_ticks, stride)):
        for name in log.agents:
            tr = tracks[name]
            if len(tr):
                j = min(i, len(tr) - 1)
                lines[name].set_data(tr[:j + 1, 1], tr[:j + 1, 2])

            ticks = node_ticks[name]
            if len(ticks):
                idx = int(np.searchsorted(ticks, i, side="right")) - 1
                if idx >= 0 and idx != node_idx[name]:
                    node_idx[name] = idx
                    est = node_est[name]
                    est_lines[name].set_data(est[:idx + 1, 0], est[:idx + 1, 1])
                    sig_xy = node_sigma[name][idx][3:5, 3:5]
                    if np.all(np.isfinite(sig_xy)):
                        e = cov_ellipse_pts(est[idx], sig_xy, nstd=2)
                        ellipses[name].set_xy(e.T)
                        ellipses[name].set_visible(True)
                    else:
                        ellipses[name].set_visible(False)

        for ev_tick, art in event_markers:
            d = i - ev_tick
            art.set_alpha(max(0.0, 1.0 - d / EVENT_FADE_FRAMES) if 0 <= d < EVENT_FADE_FRAMES else 0.0)

        fig.savefig(os.path.join(out_dir, f"{frame:06d}.png"))
        if progress_every and frame % progress_every == 0:
            print(f"  map video: frame {frame} (tick {i}/{n_ticks})")
    plt.close(fig)
    return out_dir, len(range(0, n_ticks, stride))


def fig_plan_view(log, out_dir, ax=None, title="Flown vs planned"):
    """Fig 1 -- the headline. Did it get there, and did it stay clear?"""
    import matplotlib.pyplot as plt
    use_style()
    fig, ax = (None, ax) if ax is not None else plt.subplots(figsize=(9, 5.5))

    r = float(log["agent_radius"])
    draw_obstacles(ax, log.obstacles, r)

    lm, lmc = log["landmarks"], log["landmark_cov"]
    ev = log["landmark_events"]
    detected = np.zeros(len(lm), bool)
    for lid in ev[:, 1].astype(int) if len(ev) else []:
        if 0 <= lid - 1 < len(lm):
            detected[lid - 1] = True
    draw_landmarks(ax, lm, lmc, detected=detected if len(ev) else None)
    for e in ev:                                     # realized detection rays
        lid = int(e[1]) - 1
        if 0 <= lid < len(lm):
            ax.plot([e[2], lm[lid, 0]], [e[3], lm[lid, 1]], "-", color="0.5",
                    lw=0.4, alpha=0.5, zorder=1)

    for name in log.agents:
        c = agent_color(name, log.primary)
        draw_planned_path(ax, log[f"{name}_ref"],
                          label="planned" if name == log.primary else None)
        tr = log[f"{name}_track"]
        if len(tr):
            ax.plot(tr[:, 1], tr[:, 2], "-", color=c, lw=1.6, zorder=5,
                    label=f"{name} flown" + (" (primary)" if name == log.primary else ""))
        est = log.node(name, "est")
        if len(est):
            ax.plot(est[:, 0], est[:, 1], ":", color=c, lw=1.1, alpha=0.9, zorder=5,
                    label=f"{name} FG estimate")

    for c in log["comm_events"]:
        ax.plot([c[1], c[3]], [c[2], c[4]], "-", color="0.25", lw=0.5, alpha=0.55, zorder=4)

    # True 2-sigma pose ellipses, unscaled -- these are the numbers under test.
    est, sig = log.node(log.primary, "est"), log.node(log.primary, "sigma")
    if len(est):
        step = max(1, len(est) // 14)
        for i in range(0, len(est), step):
            e = cov_ellipse_pts(est[i], sig[i][3:5, 3:5], nstd=2)
            ax.plot(e[0], e[1], "-", color=EST_COLOR, lw=0.6, alpha=0.55, zorder=5)
        ax.plot([], [], "-", color=EST_COLOR, lw=0.6, label=r"FG 2$\sigma$ (true scale)")

    tr = log[f"{log.primary}_track"]
    draw_breaches(ax, tr[:, 1:3], log[f"{log.primary}_pen"])
    draw_start_goal(ax, tr[0, 1:3] if len(tr) else log["goal_xy"], log["goal_xy"])

    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(title)
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5))
    return _finish(fig, out_dir, "fig1_plan_view") if fig else []


def fig_monte_carlo(log, agg, out_dir, nstd=2):
    """Fig 5 -- mean flown path across N Monte Carlo trials (`monte_carlo.py`),
    with a shaded cross-track std-dev tube around each agent's mean path. The
    tube follows the mean path's local normal at each point rather than
    plotting independent x/y std bands, which would be wrong for anything but
    a straight line.

    `log` is any one trial's Log -- obstacles/landmarks/planned path are
    identical across trials (same plan), so there's nothing to aggregate
    there. `agg` is `{name: (mean_xy, std_cross, normal)}` from
    `monte_carlo.aggregate`, already scaled to `nstd` standard deviations.
    """
    import matplotlib.pyplot as plt
    use_style()
    fig, ax = plt.subplots(figsize=(9, 5.5))

    r = float(log["agent_radius"])
    draw_obstacles(ax, log.obstacles, r)
    draw_landmarks(ax, log["landmarks"], log["landmark_cov"])
    for name in log.agents:
        draw_planned_path(ax, log[f"{name}_ref"], label="planned" if name == log.primary else None)

    for name, (mean_xy, std_cross, normal) in agg.items():
        c = agent_color(name, log.primary)
        band = np.vstack([mean_xy + std_cross[:, None] * normal,
                          (mean_xy - std_cross[:, None] * normal)[::-1]])
        ax.fill(band[:, 0], band[:, 1], color=c, alpha=0.3, zorder=4,
                label=rf"{name} $\pm{nstd}\sigma$ cross-track")
        ax.plot(mean_xy[:, 0], mean_xy[:, 1], "-", color=c, lw=1.8, zorder=5,
                label=f"{name} mean flown")

    primary_mean = agg[log.primary][0] if log.primary in agg else next(iter(agg.values()))[0]
    draw_start_goal(ax, primary_mean[0], log["goal_xy"])

    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title("Monte Carlo mean path")
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5))
    return _finish(fig, out_dir, "fig5_monte_carlo")


def render_uncertainty_video(sim_dir, out_dir, progress_every=500, stride=VIDEO_STRIDE):
    """Animated form of `fig_uncertainty` -- per-agent sigma vs. arc, growing
    in sync with `render_map_video`'s frame count so the two can be composited
    into one video. Unlike the map's fading event flashes, landmark/comm
    events here appear at their tick and stay -- this chart's whole point is
    showing history, so persistent markers match the static `fig_uncertainty`
    convention instead.
    """
    import matplotlib
    matplotlib.use("Agg", force=False)
    import matplotlib.pyplot as plt
    use_style()
    matplotlib.rcParams["savefig.bbox"] = None
    matplotlib.rcParams["savefig.dpi"] = "figure"
    log = Log(sim_dir)
    os.makedirs(out_dir, exist_ok=True)

    fig, ax = plt.subplots(figsize=(12.8, 3.2), dpi=100)
    fig.set_layout_engine(None)
    ax.set_position([0.06, 0.2, 0.9, 0.72])

    node_ticks, node_arc, node_unc, lines = {}, {}, {}, {}
    max_arc, max_unc = 0.0, 1.0
    for name in log.agents:
        node_ticks[name] = log.node(name, "tick").astype(int)
        node_arc[name] = log.node(name, "arc")
        node_unc[name] = log.node(name, "unc")
        if len(node_arc[name]):
            max_arc = max(max_arc, float(node_arc[name][-1]))
        finite = node_unc[name][np.isfinite(node_unc[name])]
        if len(finite):
            max_unc = max(max_unc, float(finite.max()))
        c = agent_color(name, log.primary)
        lines[name], = ax.plot([], [], "-", color=c,
                               lw=1.6 if name == log.primary else 1.0,
                               label=f"{name} (sim)" + (" primary" if name == log.primary else ""))

    thr = float(log["unc_threshold"])
    if np.isfinite(thr):
        ax.axhline(thr, color="red", ls=":", lw=1.5, label="feasibility threshold")
        max_unc = max(max_unc, thr)
    pred = float(log["planner_unc"])
    if np.isfinite(pred):
        ax.axhline(pred, color=PRIMARY_COLOR, ls="--", lw=1.2, alpha=0.7,
                   label="planner predicted terminal")
        max_unc = max(max_unc, pred)

    event_lines = []
    for e in log["landmark_events"]:
        ln = ax.axvline(e[0], color="0.6", lw=0.5, alpha=0.5, visible=False)
        event_lines.append((int(e[4]), ln))
    if len(log["landmark_events"]):
        ax.plot([], [], "-", color="0.6", lw=0.5, label="landmark detection")
    for e in log["comm_events"]:
        ln = ax.axvline(e[0], color="0.25", ls="--", lw=0.6, alpha=0.5, visible=False)
        event_lines.append((int(e[5]), ln))
    if len(log["comm_events"]):
        ax.plot([], [], "--", color="0.25", lw=0.6, label="comms")

    ax.set_xlim(0, max_arc * 1.02 if max_arc > 0 else 1.0)
    ax.set_ylim(0, max_unc * 1.15)
    ax.set_xlabel("distance travelled (m)")
    ax.set_ylabel(r"det$(\Sigma)^{1/4}$ (m)")
    ax.set_title("Realized uncertainty vs the planner's prediction")
    ax.legend(loc="upper left", ncol=2, fontsize=6)

    # n_ticks from the per-tick track, not the sparser node arrays, so this
    # matches render_map_video's frame count exactly for compositing.
    n_ticks = max((len(log[f"{name}_track"]) for name in log.agents), default=0)
    node_idx = {name: -1 for name in log.agents}
    for frame, i in enumerate(range(0, n_ticks, stride)):
        for name in log.agents:
            ticks = node_ticks[name]
            if len(ticks):
                idx = int(np.searchsorted(ticks, i, side="right")) - 1
                if idx >= 0 and idx != node_idx[name]:
                    node_idx[name] = idx
                    lines[name].set_data(node_arc[name][:idx + 1], node_unc[name][:idx + 1])
        for ev_tick, ln in event_lines:
            if i >= ev_tick:
                ln.set_visible(True)
        fig.savefig(os.path.join(out_dir, f"{frame:06d}.png"))
        if progress_every and frame % progress_every == 0:
            print(f"  uncertainty video: frame {frame} (tick {i}/{n_ticks})")
    plt.close(fig)
    return out_dir, len(range(0, n_ticks, stride))


def fig_uncertainty(log, out_dir):
    """Fig 2 -- does the planner's covariance model predict reality?"""
    import matplotlib.pyplot as plt
    use_style()
    fig, ax = plt.subplots(figsize=(7.5, 3.4))

    for name in log.agents:
        arc, unc = log.node(name, "arc"), log.node(name, "unc")
        if len(arc):
            ax.plot(arc, unc, "-", color=agent_color(name, log.primary),
                    lw=1.6 if name == log.primary else 1.0,
                    label=f"{name} (sim)" + (" primary" if name == log.primary else ""))

    thr = float(log["unc_threshold"])
    if np.isfinite(thr):
        ax.axhline(thr, color="red", ls=":", lw=1.5, label="feasibility threshold")
    pred = float(log["planner_unc"])
    if np.isfinite(pred):
        ax.axhline(pred, color=PRIMARY_COLOR, ls="--", lw=1.2, alpha=0.7,
                   label="planner predicted terminal")

    for e in log["landmark_events"]:
        ax.axvline(e[0], color="0.6", lw=0.5, alpha=0.5)
    if len(log["landmark_events"]):
        ax.plot([], [], "-", color="0.6", lw=0.5, label="landmark detection")

    for e in log["comm_events"]:
        ax.axvline(e[0], color="0.25", ls="--", lw=0.6, alpha=0.5)
    if len(log["comm_events"]):
        ax.plot([], [], "--", color="0.25", lw=0.6, label="comms")

    ax.set_xlabel("distance travelled (m)")
    ax.set_ylabel(r"det$(\Sigma)^{1/4}$ (m)")
    ax.set_title("Realized uncertainty vs the planner's prediction")
    ax.legend(loc="upper left", ncol=2)
    return _finish(fig, out_dir, "fig2_uncertainty")


def fig_error(log, out_dir):
    """Fig 3 -- did estimator error translate into path error?"""
    import matplotlib.pyplot as plt
    use_style()
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(7.5, 5.0), sharex=True)
    p = log.primary
    arc = log.node(p, "arc")

    ax.plot(arc, log.node(p, "xtrack"), "-", color=PRIMARY_COLOR, label="cross-track from plan")
    ax.plot(arc, log.node(p, "est_err"), "-", color=EST_COLOR, label=r"$\|$estimate $-$ truth$\|$")
    ax.set_ylabel("error (m)")
    ax.set_title("Tracking and estimation error along the mission")
    ax.legend(loc="upper left")

    # Obstacle proximity: 0 outside, positive inside. Shows whether the error
    # peaks anywhere it could actually have cost a collision.
    tr, pen = log[f"{p}_track"], log[f"{p}_pen"]
    s = plan_io.arclength(tr[:, 1:3]) if len(tr) > 1 else np.zeros(len(tr))
    ax2.fill_between(s, 0, pen, color=BREACH_COLOR, alpha=0.5, label="obstacle penetration")
    depth = tr[:, 3] if len(tr) else np.empty(0)
    ax3 = ax2.twinx()
    ax3.plot(s, depth, "-", color="0.4", lw=0.9, label="depth (z)")
    ax3.set_ylabel("z (m)"); ax3.grid(False)
    ax2.set_xlabel("distance travelled (m)"); ax2.set_ylabel("penetration (m)")
    ax2.legend(loc="upper left"); ax3.legend(loc="upper right")
    return _finish(fig, out_dir, "fig3_error")


def fig_ab_overlay(log_a, log_b, out_dir, labels=("truth in loop", "estimate in loop")):
    """Fig 4 -- what did closing the loop through the estimator cost?

    Only possible because the controller takes a pose argument instead of
    reading the simulator, so the two runs differ in exactly one thing.
    """
    import matplotlib.pyplot as plt
    use_style()
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 4.6), width_ratios=[3, 2])

    draw_obstacles(ax, log_a.obstacles, float(log_a["agent_radius"]))
    draw_landmarks(ax, log_a["landmarks"], log_a["landmark_cov"])
    draw_planned_path(ax, log_a[f"{log_a.primary}_ref"], label="planned")
    for lg, lab, col in ((log_a, labels[0], "#377eb8"), (log_b, labels[1], "#e41a1c")):
        tr = lg[f"{lg.primary}_track"]
        if len(tr):
            ax.plot(tr[:, 1], tr[:, 2], "-", color=col, lw=1.7, label=lab, zorder=5)
            draw_breaches(ax, tr[:, 1:3], lg[f"{lg.primary}_pen"], label=False)
    draw_start_goal(ax, log_a[f"{log_a.primary}_track"][0, 1:3], log_a["goal_xy"])
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title("Flown path"); ax.legend(loc="best")

    for lg, lab, col in ((log_a, labels[0], "#377eb8"), (log_b, labels[1], "#e41a1c")):
        ax2.plot(lg.node(lg.primary, "arc"), lg.node(lg.primary, "xtrack"), "-",
                 color=col, lw=1.4, label=lab)
    ax2.set_xlabel("distance travelled (m)"); ax2.set_ylabel("cross-track error (m)")
    ax2.set_title("Cost of closing the loop through the estimator")
    ax2.legend(loc="upper left")
    return _finish(fig, out_dir, "fig4_ab_overlay")


def render_all(sim_dir, out_dir=None, ab_dir=None):
    """Every figure the paper needs, from logs alone."""
    import matplotlib
    # Standalone, this selects Agg so figures never need a display. Called from
    # main.py the interactive backend is already up with the live panel's figure
    # open, and force-switching there would tear it out from under matplotlib --
    # force=False keeps whatever is already running, which can savefig equally well.
    matplotlib.use("Agg", force=False)
    log = Log(sim_dir)
    out_dir = out_dir or os.path.join(log.dir, "figures")
    made = []
    made += fig_plan_view(log, out_dir)
    made += fig_uncertainty(log, out_dir)
    made += fig_error(log, out_dir)
    if ab_dir:
        made += fig_ab_overlay(log, Log(ab_dir), out_dir)
    return made


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        sys.exit("usage: python viz.py <sim_dir> [ab_sim_dir]")
    for p in render_all(sys.argv[1], ab_dir=sys.argv[2] if len(sys.argv) > 2 else None):
        print("wrote", p)
