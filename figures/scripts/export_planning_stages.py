#!/usr/bin/env python3
"""The website's planning-stages replay, exported from the Fig. 1 run's logs.

    python export_planning_stages.py <video_data_dir> [<out_json>]

<video_data_dir> is the supplementary video's data folder (the one holding
fig1/). Writes website/src/data/planning-stages.json by default.

Nothing is recomputed here; the page replays what the planner logged.

Stage 1 comes from fig1/astar_trace.csv, the joint A*'s own trace. Every pop
is kept, and the joint path to it is stored as a pointer to its parent's pop,
so the browser rebuilds any path by walking back to the start. A* pops
best-first, which has no spatial order, so the pops are stored sorted by the
primary's cumulative distance g, as the video's Wavefront replays them: the
frontier then sweeps outward. Every path drawn was really popped; only WHEN it
is drawn changes. Obstacle prunes are tied to the pop being expanded.

Stage 2 comes from fig1/cont_steps.csv (length, sigma, barrier mu per kept
iterate) and fig1/cont_dense.csv, the clamped cubic B-spline the planner
evaluated at each iterate. The spline is not re-evaluated in the browser, so
no second copy of the basis can drift from the one that shipped. To keep the
page light the curve keeps every DENSE_EVERY-th sample (and the last), rounded
to the meter: about 18 m chords, under 10 px at the page's width.

Agent 1 in the logs is the support and agent 2 the primary (scene.json's
agents, and the trace's goal sigma matching primary_disc_unc).
"""
import csv
import json
import math
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(HERE, "..", "..", "website", "src", "data", "planning-stages.json")
AGENTS = {"1": "support", "2": "primary"}
DENSE_EVERY = 2


def r1(v):
    return round(float(v), 1)


def r3(v):
    return round(float(v), 3)


def main(data_dir, out_path):
    fig1 = os.path.join(data_dir, "fig1")
    with open(os.path.join(fig1, "scene.json")) as f:
        scene = json.load(f)
    with open(os.path.join(fig1, "seed.json")) as f:
        seed = json.load(f)
    with open(os.path.join(fig1, "astar_trace.csv")) as f:
        trace = list(csv.DictReader(f))

    node_xy = {int(i): (x, y) for i, x, y in scene["nodes"]}

    # --- Stage 1: pops in g order, each pointing at its parent's pop -------
    pops = [r for r in trace if r["ev"] == "pop"]
    # Ties in g keep pop order; a child is always 100 m deeper than its parent,
    # so every parent still comes before its children.
    pops.sort(key=lambda r: float(r["g"]))
    rank = {r["si"]: k for k, r in enumerate(pops)}
    stage1 = []
    for r in pops:
        a, b = (int(n) for n in r["nodes"].split(";"))
        ua, ub = r["uncs"].split(";")
        stage1.append([rank.get(r["parent"], -1), a, b, r1(r["g"]), r3(ua), r3(ub)])

    # The goal state is pushed, never popped: its parent is the last pop on the seed.
    goal = next(r for r in trace if r["ev"] == "goal")
    ga, gb = (int(n) for n in goal["nodes"].split(";"))
    k = rank[goal["parent"]]
    seed_nodes = {"support": [ga], "primary": [gb]}
    while k >= 0:
        _, a, b, *_ = stage1[k]
        seed_nodes["support"].append(a)
        seed_nodes["primary"].append(b)
        k = stage1[k][0]
    for agent in seed_nodes:
        seed_nodes[agent].reverse()
    by_flag = {a["primary"]: a for a in seed["agents"]}
    for agent, flag in (("support", False), ("primary", True)):
        want = by_flag[flag]["path"]
        got = [node_xy[n] for n in seed_nodes[agent]]
        assert len(got) == len(want) and all(math.dist(p, q) < 0.01 for p, q in zip(got, want)), agent

    # Obstacle prunes: drawn at the node of the agent whose edge hit (obstacle:<agent>).
    prunes = []
    for r in trace:
        if r["ev"] != "prune" or not r["reason"].startswith("obstacle:"):
            continue
        agent = int(r["reason"].split(":")[1])
        x, y = node_xy[int(r["nodes"].split(";")[agent - 1])]
        prunes.append([rank[r["parent"]], r1(x), r1(y)])
    prunes.sort(key=lambda p: p[0])

    used = sorted({n for p in stage1 for n in p[1:3]} | {ga, gb})
    nodes = {str(n): [r1(node_xy[n][0]), r1(node_xy[n][1])] for n in used}

    # --- Stage 2: kept iterates of the refinement ---------------------------
    with open(os.path.join(fig1, "cont_steps.csv")) as f:
        steps = list(csv.DictReader(f))
    dense = defaultdict(lambda: defaultdict(list))
    with open(os.path.join(fig1, "cont_dense.csv")) as f:
        for r in csv.DictReader(f):
            dense[int(r["iter"])][AGENTS[r["agent"]]].append((int(r["i"]), r1(r["x"]), r1(r["y"])))
    ctrls = defaultdict(lambda: defaultdict(list))
    with open(os.path.join(fig1, "cont_ctrls.csv")) as f:
        for r in csv.DictReader(f):
            ctrls[int(r["iter"])][AGENTS[r["agent"]]].append((int(r["ctrl_index"]), r1(r["x"]), r1(r["y"])))

    def flat(rows):
        return [v for _, x, y in sorted(rows) for v in (x, y)]

    def thin(rows):
        rows = sorted(rows)
        kept = rows[::DENSE_EVERY] + ([rows[-1]] if (len(rows) - 1) % DENSE_EVERY else [])
        return [round(v) for _, x, y in kept for v in (x, y)]

    stage2 = []
    for s in steps:
        it = int(s["iter"])
        stage2.append({
            "iter": it,
            "stage": int(s["stage"]),
            "mu": float(f"{float(s['mu']):.4g}"),
            "length": r1(s["len"]),
            "sigma": r3(s["unc"]),
            "path": {a: thin(dense[it][a]) for a in ("primary", "support")},
            "ctrls": {a: flat(ctrls[it][a]) for a in ("primary", "support")},
        })

    # --- Static map ----------------------------------------------------------
    hex_r = scene["hex"]["radius"]
    cx = [c[0] for c in scene["hex"]["centers"]]
    cy = [c[1] for c in scene["hex"]["centers"]]
    pad = hex_r + 4
    landmarks = []
    for lm in scene["landmarks"]:
        # Drawn as Fig. 2 and the ladder draw them: 2 sigma of 400x the covariance.
        (sxx, sxy), (_, syy) = lm["cov"]
        tr, det = sxx + syy, sxx * syy - sxy * sxy
        l1 = tr / 2 + math.sqrt(max(tr * tr / 4 - det, 0))
        l2 = tr / 2 - math.sqrt(max(tr * tr / 4 - det, 0))
        angle = math.degrees(0.5 * math.atan2(2 * sxy, sxx - syy))
        landmarks.append({"x": lm["x"], "y": lm["y"], "rx": r1(2 * math.sqrt(400 * l1)),
                          "ry": r1(2 * math.sqrt(400 * l2)), "angle": r1(angle)})

    out = {
        "_source": "The Fig. 1 run's logs (supplementary video data, fig1/): astar_trace.csv, cont_steps.csv, cont_dense.csv, cont_ctrls.csv, scene.json.",
        "_note": "stage1.pops are sorted by the primary's g, not pop order, and are [parent rank, support node, primary node, g, support sigma, primary sigma]. Paths and control points are flat [x0, y0, x1, y1, ...].",
        "units": "m",
        "view": {"x0": r1(min(cx) - pad), "x1": r1(max(cx) + pad), "y0": r1(min(cy) - pad), "y1": r1(max(cy) + pad)},
        "start": scene["start"],
        "goal": [r1(v) for v in scene["goal"]],
        "bound": float(scene["cfg"]["unc_radius_threshold"]),
        "hex": {"radius": r3(hex_r), "centers": [[r1(x), r1(y)] for x, y in scene["hex"]["centers"]]},
        "obstacles": scene["obstacles"],
        "landmarks": landmarks,
        "nodes": nodes,
        "stage1": {
            "pops": stage1,
            "prunes": prunes,
            "seed": {a: [v for n in seed_nodes[a] for v in nodes[str(n)]] for a in ("primary", "support")},
            "seedLength": r1(goal["g"]),
            "seedSigma": r3(goal["uncs"].split(";")[1]),
        },
        "stage2": stage2,
    }
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    print(f"{out_path}: {len(stage1)} pops, {len(prunes)} obstacle prunes, {len(stage2)} iterates, "
          f"{os.path.getsize(out_path) / 1024:.0f} KB")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT)
