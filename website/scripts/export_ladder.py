"""Build the constraint ladder's data from the planner's own Fig. 2 runs.

    python3 scripts/export_ladder.py        (from website/; no dependencies)

Reads the four plans behind Fig. 2 in code/results/paper/ladder/hexspline_cl/
and writes src/data/constraint-ladder.json. For each level and agent it stores:

  waypoints  the B-spline's control points. Fig. 2's configs set
             cont_unc_use_waypoints: true, so the planner propagates covariance
             and schedules exchanges along this polygon. The replay computes its
             exchange and landmark weights here, as the planner did.
  path       the flown trajectory, the clamped cubic B-spline (a port of the
             planner's bspline_sample_path), sampled every PATH_STEP metres of
             waypoint arc at the matching spline point (via Greville abscissae).
             The replay moves the agents along it on the planner's clock.

Each level also stores the exchanges the planner scheduled, computed here at
full precision with covariance.jl's rule (apply_synchronized_propagation): a
checkpoint every comm_interval of waypoint arc up to the longer path's end,
weighted by comm_weight and kept when above comm_weight_min. The script checks
its spline against each run's results.yaml before writing.
"""

import csv
import json
import math
import re
from pathlib import Path

SITE = Path(__file__).resolve().parent.parent
RUNS = SITE.parent / 'code' / 'results' / 'paper' / 'ladder' / 'hexspline_cl'
OUT = SITE / 'src' / 'data' / 'constraint-ladder.json'
PATH_STEP = 10.0
DEGREE = 3
LENGTH_SAMPLES_PER_SEG = 5  # the planner's spline_samples_per_seg, for the length check

# Panel letter and Fig. 2's caption for each level.
LEVELS = {
    100: ('a', 'The support climbs into the north pocket for the weak landmark there and rejoins.'),
    70: ('b', 'The support takes the southern lane to a stronger landmark and relays it.'),
    50: ('c', 'The primary adds a turn before the goal so that the support can reach the second landmark.'),
    30: ('d', 'The primary dips to the center line to stall, allowing the support to relay from a closer distance.'),
}


def flat_yaml(path):
    """The flat `key: value` pairs of a planner config or results file."""
    out = {}
    for line in path.read_text().splitlines():
        m = re.match(r'\s*([A-Za-z_][\w]*):\s*([^#\[]+?)\s*(#.*)?$', line)
        if m:
            out[m.group(1)] = m.group(2)
    return out


# --- The planner's B-spline (src/planner/algorithms/hexspline_cl.jl) ---------

def knots(n):
    spans = max(1, n - DEGREE)
    return [0.0 if i <= DEGREE + 1 else 1.0 if i > n else (i - DEGREE - 1) / spans for i in range(1, n + DEGREE + 2)]


def find_span(u, k, n):
    if u >= k[-1 - DEGREE]:
        return n
    lo, hi = DEGREE + 1, n
    while lo <= hi:
        mid = (lo + hi) // 2
        if u < k[mid - 1]:
            hi = mid - 1
        elif u >= k[mid]:
            lo = mid + 1
        else:
            return mid
    return min(max(lo, DEGREE + 1), n)


def point(ctrl, u):
    n, k = len(ctrl), knots(len(ctrl))
    s = find_span(u, k, n)
    N, left, right = [1.0] + [0.0] * DEGREE, [0.0] * DEGREE, [0.0] * DEGREE
    for j in range(1, DEGREE + 1):
        left[j - 1], right[j - 1], saved = u - k[s - j], k[s + j - 1] - u, 0.0
        for r in range(1, j + 1):
            den = right[r - 1] + left[j - r]
            t = 0.0 if abs(den) < 1e-14 else N[r - 1] / den
            N[r - 1], saved = saved + right[r - 1] * t, left[j - r] * t
        N[j] = saved
    first = s - DEGREE - 1
    return tuple(sum(N[j] * ctrl[first + j][c] for j in range(DEGREE + 1)) for c in (0, 1))


def spline_length(ctrl):
    m = max(1, len(ctrl) - DEGREE) * LENGTH_SAMPLES_PER_SEG + 1
    pts = [point(ctrl, i / (m - 1)) for i in range(m)]
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(m - 1))


def path_on_waypoint_clock(ctrl):
    """Spline points at every PATH_STEP m of waypoint arc, plus the end."""
    k = knots(len(ctrl))
    greville = [sum(k[i + 1:i + 1 + DEGREE]) / DEGREE for i in range(len(ctrl))]
    cum = [0.0]
    for a, b in zip(ctrl, ctrl[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    stops = [i * PATH_STEP for i in range(int(cum[-1] // PATH_STEP) + 1)]
    if cum[-1] - stops[-1] > 1e-6:
        stops.append(cum[-1])
    out, seg = [], 0
    for s in stops:
        while seg < len(ctrl) - 2 and cum[seg + 1] < s:
            seg += 1
        span = cum[seg + 1] - cum[seg]
        t = 0.0 if span < 1e-12 else min(max((s - cum[seg]) / span, 0.0), 1.0)
        out.append(point(ctrl, greville[seg] + t * (greville[seg + 1] - greville[seg])))
    return out, cum[-1]


def at_arc(ctrl, s):
    """The point at arc length s along the waypoint polygon, parked at its end."""
    acc = 0.0
    for a, b in zip(ctrl, ctrl[1:]):
        seg = math.dist(a, b)
        if seg > 0 and acc + seg >= s:
            t = (s - acc) / seg
            return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        acc += seg
    return ctrl[-1]


def exchanges(primary, support, model, end):
    """The planner's checkpoints (s > 0) and their comm weights."""
    out, k = [], 1
    while k * model['comm_interval'] <= end + 1e-6:
        s = k * model['comm_interval']
        d = math.dist(at_arc(primary, s), at_arc(support, s))
        w = 1.0 / (1.0 + math.exp((d - model['comm_range']) / model['comm_width']))
        if w > model['comm_weight_min']:
            out.append([round(s, 2), round(w, 4)])
        k += 1
    return out


# --- Assembly -------------------------------------------------------------------

def r1(v):
    return round(v, 1)


def main():
    trials = {int(r['pct']): r for r in csv.DictReader((RUNS / 'trials.csv').open())}
    first = RUNS / 's001_p100'
    cfg = flat_yaml(first / '_cfg' / 'main.yaml')
    model = {k: float(cfg[k]) for k in ('comm_interval', 'comm_range', 'comm_width', 'comm_weight_min',
                                         'visibility_range', 'visibility_width')}

    obstacles = {}
    for r in csv.DictReader((first / 'scenario_obstacles.csv').open()):
        obstacles.setdefault(r['obstacle_id'], []).append([r1(float(r['x'])), r1(float(r['y']))])
    landmarks = []
    for r in csv.DictReader((first / 'scenario_landmarks.csv').open()):
        a, b, c = float(r['c11']), float(r['c12']), float(r['c22'])
        # Fig. 2's cosmetic ellipse: 2 sigma of 400x the map covariance, so 40 sigma.
        mean, half = (a + c) / 2, math.hypot((a - c) / 2, b)
        landmarks.append({'x': r1(float(r['x'])), 'y': r1(float(r['y'])),
                          'rx': r1(40 * math.sqrt(mean + half)), 'ry': r1(40 * math.sqrt(max(mean - half, 0.0))),
                          'angle': r1(math.degrees(0.5 * math.atan2(2 * b, a - c)))})

    levels, goal = [], None
    for pct, (panel, note) in LEVELS.items():
        run = RUNS / f's001_p{pct:03d}'
        agents = {}
        for r in csv.DictReader((run / 'csv' / 'main_ctrls.csv').open()):
            agents.setdefault(int(r['agent']), []).append((float(r['x']), float(r['y'])))
        support, primary = agents[1], agents[max(agents)]  # the last agent is the primary
        results = flat_yaml(run / 'results.yaml')
        length = float(results['primary_length'])
        if abs(spline_length(primary) - length) > 1e-3:
            raise SystemExit(f'{run.name}: spline length {spline_length(primary):.4f} != results.yaml {length:.4f}')
        goal = primary[-1]
        entry = {'panel': panel, 'level': pct, 'bound': round(float(trials[pct]['threshold']), 2),
                 'length': round(length), 'sigma': round(float(results['primary_unc']), 2), 'note': note}
        ends = []
        for name, ctrl in (('primary', primary), ('support', support)):
            path, waypoint_length = path_on_waypoint_clock(ctrl)
            ends.append(waypoint_length)
            entry[name] = {'waypoints': [[round(x, 2), round(y, 2)] for x, y in ctrl],
                           'path': [[r1(x), r1(y)] for x, y in path],
                           'waypointLength': round(waypoint_length, 2)}
        entry['exchanges'] = exchanges(primary, support, model, max(ends))
        # The clock ends with the longer path, and never before its last checkpoint.
        entry['end'] = round(max([*ends, *(s for s, _ in entry['exchanges'])]), 2)
        levels.append(entry)

    y_max = float(cfg.get('corridor_y_max_m', 300.0))
    data = {
        '_source': 'The four Fig. 2 runs in code/results/paper/ladder/hexspline_cl, via website/scripts/export_ladder.py.',
        'units': 'm',
        'view': {'x0': -60, 'x1': round(goal[0] + 60), 'y0': round(-y_max - 45), 'y1': round(y_max + 45)},
        'start': [r1(v) for v in levels[0]['primary']['waypoints'][0]],
        'goal': [r1(v) for v in goal],
        'pathStep': PATH_STEP,
        'model': model,
        'obstacles': list(obstacles.values()),
        'landmarks': landmarks,
        'levels': levels,
    }
    OUT.write_text(json.dumps(data, separators=(',', ':')))
    print(f'{OUT.relative_to(SITE)}  {OUT.stat().st_size // 1024} KB')
    for l in levels:
        tail = ', '.join(f'{s:.0f}: {w:.2f}' for s, w in l['exchanges'][-3:])
        print(f"  ({l['panel']}) {l['level']}%: {l['length']} m, sigma {l['sigma']} m, bound {l['bound']} m; last exchanges {tail}")


if __name__ == '__main__':
    main()
