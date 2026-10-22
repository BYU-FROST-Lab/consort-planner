"""Waypoint follower for a torpedo AUV under HoloOcean's Fossen autopilot.

Ported from the working prototype at holoocean/waypointtest.py:157-213, with one
change: the pose comes in as an argument instead of being read from the
simulator. That is what makes "steer on the factor-graph estimate" vs "steer on
ground truth" a one-line swap at the call site rather than a forked loop.


This is an OUTER loop only. It emits (depth, heading, rpm) goals; the inner loop
is HoloOcean's own `depthHeadingAutopilot` (integral sliding-mode heading + PID
depth, holoocean/fossen_dynamics/control.py), tuned by the `autopilot` block in
the scenario dict. There is no PID here on purpose.

Two behaviours look fussy and are not -- both were arrived at empirically
against a vehicle with a 40 m turn radius:

  * Closest-approach arrival. Declaring arrival on `distance < R` alone gives
    false positives the moment the vehicle overshoots; instead track the closest
    approach so far and only commit once the vehicle is also moving away.
  * Heading re-arm hysteresis. Re-aiming at the waypoint every update makes the
    vehicle oscillate, because early in a turn the bearing to the waypoint keeps
    swinging. Commit to a heading on a fresh waypoint and only re-aim once
    roughly aligned. Note this is now largely inert: with a non-zero
    `lookahead_m` the command no longer swings enough to trip the 30 deg
    threshold, and sweeping it from 30 to 180 deg changes nothing. It is kept
    because it still matters at lookahead_m=0.
"""

import numpy as np


def wrap180(deg):
    """Signed angle difference folded into [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


class WaypointFollower:
    """Drive one agent through a list of [x, y] waypoints at a fixed depth.

    `update` is called once per tick (main.py:378), on a belief dead-reckoned
    forward between factor-graph fixes -- not once per fix. Steering at the
    estimator's 5 m cadence leaves the autopilot chasing a 2.3 s-old command;
    see the cadence note in ARCHITECTURE.md. `update_spacing_m` is therefore the tick
    spacing, and only guards the arrival ball against being stepped over.
    """

    # arrival_radius is a calibration knob against the vehicle's turn radius, not
    # a free parameter. A vehicle held to a 40 m turn radius cuts corners, so on a
    # tight bend its closest approach to the waypoint can exceed a small ball --
    # `min_dist < R` then never fires and the leg ends via the stall guard, far off
    # track. Measured on a 1/kappa = 82 m reference (see _tuning_probe below):
    # R = 8 m (waypointtest.py's value) gives 69.7 m cross-track and a skipped
    # waypoint; R = 15 m gives 10.4 m and none. Raise it with the turn radius.
    # lookahead_m is the second calibration knob, and the one that sets how much
    # the vehicle wiggles. Aiming straight at the next waypoint makes the
    # commanded bearing swing as 1/distance: at 2 m off track the bearing error
    # is 8 deg from 15 m out but 22 deg from 5 m out, so the command lurches
    # just before arrival and snaps again at the handover.
    #
    # Measure the COMMAND, not the track. On results/2026-08-14_16-45-13/mc1 the
    # commanded heading turned 2.05 deg/m with 65 reversals against a plan that
    # asks for 0.166; the flown track turned only 0.77 because the autopilot's
    # reference model filters the command, and it pays 17.5 deg RMS tracking
    # error to do it (13% of nodes past the 30 deg re-arm threshold). Judging
    # this on the track hides the fault in the thing downstream of it -- the
    # track looks merely 4x too busy while the command is 12x.
    #
    # Aiming instead at a point `lookahead_m` along the remaining polyline bounds
    # that gain at 1/lookahead_m, which takes the command to ~0.15 deg/m. That is
    # lookahead-based line-of-sight guidance, the standard path-following law in
    # Fossen 2021 (Lekkas & Fossen) -- the same book this vehicle's dynamics and
    # autopilot come from, where the lookahead distance is the one tuning knob.
    #
    # It DEFAULTS OFF (0 = aim straight at the waypoint) on purpose. The stock
    # behaviour is a faithful port of HoloOcean's own example.py, and the
    # contribution under test here is the planner, not the controller: the
    # tester is more defensible flying the simulator's shipped follower than a
    # better one of our own. The knob exists so a run can be repeated with LOS
    # guidance and the verdict shown to be insensitive to the guidance law --
    # see LOOKAHEAD_M in main.py. If the two disagree, that is a finding about
    # how much controller artefact is leaking into the verdict, and it belongs
    # in the writeup rather than being silently tuned away.
    #
    # Bigger is smoother but cuts corners: the carrot chords across bends, so
    # worst-case cross-track grows roughly linearly past ~20 m, and cross-track
    # is what the obstacle verdict is chance-constrained on. 25 m (the waypoint
    # spacing) is the value characterised below; 40 m (the turn radius) buys
    # little more smoothing for markedly worse corner-cutting.
    def __init__(self, waypoints, depth, arrival_radius=15.0, heading_rearm_deg=30.0,
                 speed_rpm=1300, update_spacing_m=5.0, max_leg_m=400.0,
                 lookahead_m=0.0):
        self.waypoints = np.asarray(waypoints, dtype=float)[:, :2]
        self.depth = float(depth)
        self.arrival_radius = float(arrival_radius)
        self.heading_rearm_deg = float(heading_rearm_deg)
        self.speed_rpm = float(speed_rpm)
        self.max_leg_m = float(max_leg_m)
        self.lookahead_m = float(lookahead_m)

        # Arrival is only sampled when update() is called. If the vehicle can
        # travel further than the arrival ball between samples it can pass
        # clean through without ever registering, and the leg only ends via the
        # stall guard below -- which reads as a controller failure that is
        # really a cadence bug. Catch it here instead.
        if arrival_radius <= update_spacing_m:
            raise ValueError(
                f"arrival_radius ({arrival_radius} m) must exceed the pose update spacing "
                f"({update_spacing_m} m), or waypoints get skipped rather than reached.")

        self.idx = 0
        self.target_heading = 0.0
        self._min_dist = np.inf
        self._leg_travel = 0.0
        self._last_pos = None
        self.reached = []   # indices arrived at properly
        self.skipped = []   # indices abandoned by the stall guard

    @property
    def done(self):
        return self.idx >= len(self.waypoints)

    @property
    def current_waypoint(self):
        return None if self.done else self.waypoints[self.idx]

    def aim_point(self, pos):
        """The point `lookahead_m` along the remaining waypoint polyline.

        Distance is measured from `pos` through the *remaining* waypoints, so
        the carrot slides forward continuously as the vehicle moves instead of
        jumping at each handover. Past the end of the list it clamps to the
        goal, which lets the final leg close normally.
        """
        if self.lookahead_m <= 0.0:
            return self.waypoints[self.idx]
        remaining, prev = self.lookahead_m, np.asarray(pos, float)[:2]
        for wp in self.waypoints[self.idx:]:
            d = float(np.linalg.norm(wp - prev))
            if d >= remaining:
                return prev + (wp - prev) * (remaining / d)
            remaining -= d
            prev = wp
        return self.waypoints[-1]

    def update(self, pos_xy, vel_xy, heading_deg):
        """Advance the follower. Returns (depth, heading_deg, rpm), or None when done.

        pos_xy      current position estimate (or truth) in the world frame, m
        vel_xy      world-frame planar velocity, m/s -- used for overshoot detection
        heading_deg current yaw, degrees, same convention as set_goal (-180..180)
        """
        if self.done:
            return None

        pos = np.asarray(pos_xy, float)[:2]
        vel = np.asarray(vel_xy, float)[:2]
        wp = self.waypoints[self.idx]
        to_wp = wp - pos

        if self._last_pos is not None:
            self._leg_travel += float(np.linalg.norm(pos - self._last_pos))
        self._last_pos = pos

        # Steer at the lookahead point, but judge arrival on the waypoint itself.
        to_aim = self.aim_point(pos) - pos

        # Commit a heading on a fresh waypoint, then re-aim once roughly aligned.
        if self._min_dist == np.inf or abs(wrap180(self.target_heading - heading_deg)) < self.heading_rearm_deg:
            self.target_heading = float(np.degrees(np.arctan2(to_aim[1], to_aim[0])))

        dist = float(np.linalg.norm(to_wp))
        self._min_dist = min(self._min_dist, dist)
        moving_away = float(np.dot(vel, to_wp)) < 0.0

        arrived = self._min_dist < self.arrival_radius and moving_away
        # Stall guard: without it an unreachable waypoint hangs the run forever
        # instead of reporting a failure, which is useless for an unattended
        # tester. Abandoning the leg is recorded and scored, not swallowed.
        stalled = self._leg_travel > self.max_leg_m

        if arrived or stalled:
            (self.reached if arrived else self.skipped).append(self.idx)
            self.idx += 1
            self._min_dist = np.inf
            self._leg_travel = 0.0
            if self.done:
                return None
            nxt = self.aim_point(pos) - pos
            self.target_heading = float(np.degrees(np.arctan2(nxt[1], nxt[0])))

        return self.depth, self.target_heading, self.speed_rpm


TURN_RADIUS_M = 40.0   # config/main.yaml: min_turn_radius_m


def _max_curvature(pts):
    """Discrete curvature of a polyline, via the circumradius of each triple."""
    a, b, c = pts[:-2], pts[1:-1], pts[2:]
    ab = np.hypot(*(b - a).T)
    bc = np.hypot(*(c - b).T)
    ca = np.hypot(*(a - c).T)
    cross = np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1])
                   - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))
    denom = ab * bc * ca
    return float(np.max(np.where(denom > 1e-12, 2.0 * cross / np.maximum(denom, 1e-12), 0.0)))


def _turn_per_metre(pts, stride=5):
    """Total |heading change| per metre travelled -- the wiggle metric.

    Compared against the same number for the reference path: a follower that
    tracks without wiggling turns about as much as the path itself asks for.
    Differencing consecutive tick positions (0.073 m apart) is dominated by
    quantisation, so sample every `stride` ticks; the value is flat for any
    stride from 5 up.
    """
    seg = np.diff(np.asarray(pts, float)[::stride], axis=0)
    s = float(np.hypot(*seg.T).sum())
    dpsi = np.degrees(np.diff(np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))))
    return float(np.abs(dpsi).sum() / max(s, 1e-9))


HEADING_TAU_S = 2.0    # see _fly

def _fly(wps, ref, speed=2.2, dt=1.0 / 30.0, tau=HEADING_TAU_S, **kw):
    """Unicycle with a lagged, rate-limited heading. Returns (follower, track).

    speed/dt default to the production loop -- main.py:378 calls update() every
    tick at 30 Hz and ~2.2 m/s, i.e. 0.073 m apart. This used to default to
    dt=3.33 s (5 m per update, the estimator cadence), which is a regime the
    driver never runs.

    `tau` lumps the autopilot's reference model and the hull's yaw response into
    one first-order lag; ~2 s is the value fitted against
    results/2026-08-14_16-45-13/mc1. It is here so the follower sees a vehicle
    that cannot turn instantly -- an instant-turn unicycle tracks a lurching
    command perfectly and hides the very fault this harness exists to catch.

    What is validated against that run is `cmds`, not `track`. On the recorded
    geometry this loop produces 64 command reversals against the log's 65, so
    the command mechanism is faithful. The track is NOT: this model lacks the
    real refModel3, so it smooths differently and lands at 0.17 deg/m where the
    log flew 0.77. Assert on the command; treat the track here as indicative
    only, and settle cross-track questions in the simulator.
    """
    f = WaypointFollower(wps, depth=240.0, update_spacing_m=speed * dt, **kw)
    pos, heading = np.array(ref[0], float), float(np.degrees(np.arctan2(*(ref[1] - ref[0])[::-1])))
    max_turn = np.degrees(speed * dt / TURN_RADIUS_M)
    track, cmds = [pos.copy()], []
    for _ in range(200_000):
        vel = speed * np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading))])
        goal = f.update(pos, vel, heading)
        if goal is None:
            break
        cmds.append(goal[1])
        heading += np.clip(wrap180(goal[1] - heading) * dt / tau, -max_turn, max_turn)
        pos = pos + vel * dt
        track.append(pos.copy())
    return f, np.array(track), np.array(cmds)


def _cmd_turn_per_metre(cmds, speed=2.2, dt=1.0 / 30.0):
    """Commanded |heading change| per metre -- what the follower is judged on.

    The track is a filtered version of this (the autopilot's reference model
    smooths it), so measuring the track flatters the follower and hides an
    oscillating command behind a merely-busy path.
    """
    c = np.degrees(np.unwrap(np.radians(np.asarray(cmds, float))))
    return float(np.abs(np.diff(c)).sum() / max(speed * dt * len(c), 1e-9))


def _self_check():
    """Fly a kinematic unicycle along a curvature-feasible spline and assert it
    visits every waypoint in order. Catches sign errors in the heading
    convention and in the overshoot test, which are what silently break this.

    The reference is built the way the planner builds one -- control points
    smoothed into a clamped cubic B-spline -- and its curvature is asserted
    feasible first, so a failure here is the follower's fault and not the
    path's."""
    import plan_io

    ctrl = np.array([[0., 0.], [100., 0.], [200., 0.], [300., -50.],
                     [400., -100.], [500., -100.], [600., -100.]])
    ref = plan_io.sample_spline(ctrl, samples_per_seg=40)
    kappa = _max_curvature(ref)
    assert kappa <= 1.0 / TURN_RADIUS_M, (
        f"test reference is not flyable: kappa={kappa:.4f} > {1 / TURN_RADIUS_M:.4f}")

    wps = plan_io.resample_by_arclength(ref, 25.0)
    f, track, cmds = _fly(wps, ref)

    assert f.done, f"never finished: stuck at waypoint {f.idx}/{len(wps)}"
    assert not f.skipped, f"stall guard fired on {f.skipped} -- the follower could not track"
    assert f.reached == list(range(len(wps))), f"out of order: {f.reached}"

    err = max(plan_io.cross_track_error(p, ref) for p in track)
    assert err < TURN_RADIUS_M, f"max cross-track {err:.1f} m exceeds the turn radius"
    print(f"tracked {len(wps)} waypoints in order (1/kappa = {1 / kappa:.0f} m), "
          f"max cross-track {err:.1f} m")

    # The default above is the stock aim-at-the-waypoint follower, so the
    # numbers it produces are a property of HoloOcean's example, not something
    # to assert a quality bound on -- pinning "the command is smooth" here would
    # just fail against the very behaviour we deliberately ship.
    #
    # What IS worth pinning is that the LOS knob still does what it was
    # characterised as doing, since the A/B in main.py is only meaningful if the
    # two arms actually differ. On results/2026-08-14_16-45-13/mc1 the stock
    # command turned 2.05 deg/m with 65 reversals against a plan asking 0.166;
    # lookahead_m=25 takes that to ~0.15.
    planned = _turn_per_metre(ref, stride=1)
    stock_turn = _cmd_turn_per_metre(cmds)
    los, los_track, los_cmds = _fly(wps, ref, lookahead_m=25.0)
    los_turn = _cmd_turn_per_metre(los_cmds)
    los_err = max(plan_io.cross_track_error(p, ref) for p in los_track)

    assert los.done and not los.skipped, (
        f"LOS arm failed to fly the path: skipped {los.skipped}")
    assert los_turn < 0.5 * stock_turn, (
        f"the LOS knob is inert: {los_turn:.2f} deg/m vs {stock_turn:.2f} stock -- "
        f"the A/B in main.py would compare two identical runs")
    print(f"commanded turning: stock {stock_turn:.2f} deg/m, "
          f"lookahead_m=25 {los_turn:.2f}, path asks {planned:.2f}")
    print(f"  cross-track: stock {err:.1f} m, lookahead_m=25 {los_err:.1f} m")

    # The stall guard must actually fire on something unflyable, or it is not a
    # guard -- an unreachable waypoint would hang an unattended run forever.
    spike = np.array([[0., 0.], [100., 0.], [100., 400.], [200., 0.], [300., 0.]])
    g, _, _ = _fly(spike, spike)
    assert g.done and g.skipped, "stall guard never fired on an unflyable path"
    print(f"stall guard abandoned waypoints {g.skipped} on an unflyable path")

    # Regression on the arrival-radius default: a tight bend is exactly where an
    # 8 m ball fails, so pin that the shipped default still handles it.
    tight = plan_io.sample_spline(
        np.array([[0., 0.], [50., 0.], [100., 0.], [150., -50.],
                  [200., -100.], [250., -100.], [300., -100.]]), samples_per_seg=40)
    tight_wps = plan_io.resample_by_arclength(tight, 25.0)
    h, h_track, _ = _fly(tight_wps, tight)
    h_err = max(plan_io.cross_track_error(p, tight) for p in h_track)
    assert not h.skipped and h_err < TURN_RADIUS_M, (
        f"default arrival_radius fails on 1/kappa={1 / _max_curvature(tight):.0f} m: "
        f"cross-track {h_err:.1f} m, skipped {h.skipped}")
    print(f"tight bend (1/kappa = {1 / _max_curvature(tight):.0f} m): "
          f"max cross-track {h_err:.1f} m, none skipped\nOK")


if __name__ == "__main__":
    _self_check()
