"""Regenerate the site's figures and constraint-ladder data from the paper PDF.

    pip install pymupdf pillow
    python scripts/extract_figures.py path/to/paper.pdf

Writes, relative to website/:
  src/assets/figures/fig*.webp     the paper's figures (lossless WebP, about 2x
                                   their on-page width; fig7 is a photo, lossy)
  src/data/constraint-ladder.json  Fig. 2's scenario and trajectories, read from
                                   the figure's vector paths

Only needed when the figures change, e.g. for the camera-ready PDF. It relies on
the figures' captions ("Fig. N.") and on Fig. 2's plotting colors; if either
changes, update FIGURES or the COLOR constants below. Fig. 2's table (level,
bound, length, terminal sigma) is an image in the PDF, so it is copied into
LEVELS by hand.
"""

import json
import math
import sys
from pathlib import Path

import pymupdf
from PIL import Image, ImageOps

SITE = Path(__file__).resolve().parent.parent
FIG_DIR = SITE / 'src' / 'assets' / 'figures'
DATA = SITE / 'src' / 'data' / 'constraint-ladder.json'
DPI = 500

# (output name, caption label). Fig. 2 is rebuilt as an SVG on the site.
FIGURES = [
    ('fig1-escort-problem', 'Fig. 1.'),
    ('fig3-baselines', 'Fig. 3.'),
    ('fig4-runtime', 'Fig. 4.'),
    ('fig5-ablation', 'Fig. 5.'),
    ('fig6-ablation-runtime', 'Fig. 6.'),
    ('fig8-monte-carlo', 'Fig. 8.'),
]
PHOTO = ('fig7-holoocean', 'Fig. 7.')

# Fig. 2's table and caption, by panel.
LEVELS = {
    'a': (100, 13.75, 1741, 9.73, 'The support climbs into the north pocket for the weak landmark there and rejoins.'),
    'b': (70, 9.62, 1761, 8.89, 'The support takes the southern lane to a stronger landmark and relays it.'),
    'c': (50, 6.87, 1822, 6.66, 'The primary adds a turn before the goal so that the support can reach the second landmark.'),
    'd': (30, 4.12, 1890, 3.91, 'The primary dips to the center line to stall, allowing the support to relay from a closer distance.'),
}

# Fig. 2's plotting colors (RGB, 0-1, rounded to 3 places).
OBSTACLE = (0.642, 0.642, 0.642)
DISC = (1.0, 0.82, 0.82)
BLACK = (0.0, 0.0, 0.0)
START = (0.0, 0.502, 0.0)
GOAL = (1.0, 0.647, 0.0)
PRIMARY = (0.0, 0.0, 1.0)
SUPPORT = (0.502, 0.0, 0.502)


def rgb(c):
    return tuple(round(v, 3) for v in c) if c else None


def caption(page, label):
    """Bounding box of the caption line that starts with `label`."""
    for block in page.get_text('dict')['blocks']:
        for line in block.get('lines', []):
            for span in line['spans']:
                if span['text'].strip().startswith(label):
                    return pymupdf.Rect(span['bbox'])
    raise SystemExit(f'caption {label!r} not found')


def find_figure(doc, label):
    for page in doc:
        try:
            return page, caption(page, label)
        except SystemExit:
            continue
    raise SystemExit(f'caption {label!r} not found in the PDF')


def figure_rect(page, cap):
    """Union of everything drawn in the caption's column, between the page top and the caption."""
    column = pymupdf.Rect(cap.x0 - 2, 40, cap.x1 + 2, cap.y0 - 0.5)
    rect = None
    for kind, bbox in page.get_bboxlog():
        r = pymupdf.Rect(bbox)
        if kind.startswith(('clip', 'ignore')) or r.is_empty or r.is_infinite:
            continue
        inter = r & column
        if not inter.is_empty and abs(inter) >= 0.9 * abs(r):
            rect = pymupdf.Rect(r) if rect is None else rect | r
    if rect is None:
        raise SystemExit('nothing drawn above the caption')
    # Pad, but never into the caption.
    return pymupdf.Rect(rect.x0 - 2, rect.y0 - 2, rect.x1 + 2, min(rect.y1 + 2, cap.y0 - 1))


def trim(img, margin=12):
    """Crop to the non-white content plus a fixed margin, so every figure has the same border."""
    box = ImageOps.invert(img.convert('L')).point(lambda v: 255 if v > 5 else 0).getbbox()
    if not box:
        return img
    x0, y0, x1, y1 = box
    return img.crop((max(x0 - margin, 0), max(y0 - margin, 0), min(x1 + margin, img.width), min(y1 + margin, img.height)))


def save_webp(pix, path, lossless=True):
    img = Image.frombytes('RGB', (pix.width, pix.height), pix.samples)
    if lossless:
        img = trim(img)
        img.save(path, 'WEBP', lossless=True, quality=100, method=6)
    else:
        img.save(path, 'WEBP', quality=80, method=6)
    print(f'{path.relative_to(SITE)}  {img.width}x{img.height}  {path.stat().st_size // 1024} KB')


def export_figures(doc):
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    for name, label in FIGURES:
        page, cap = find_figure(doc, label)
        pix = page.get_pixmap(clip=figure_rect(page, cap), dpi=DPI, alpha=False)
        save_webp(pix, FIG_DIR / f'{name}.webp')
    # The HoloOcean render is embedded as a bitmap: take it at native resolution.
    name, label = PHOTO
    page, cap = find_figure(doc, label)
    best = None
    for info in page.get_image_info(xrefs=True):
        r = pymupdf.Rect(info['bbox'])
        if r.y1 <= cap.y0 + 1 and abs(r.x0 - cap.x0) < 20 and (best is None or info['width'] > best['width']):
            best = info
    if best is None:
        raise SystemExit('no image above the Fig. 7 caption')
    pix = pymupdf.Pixmap(doc, best['xref'])
    if pix.n - pix.alpha != 3:
        pix = pymupdf.Pixmap(pymupdf.csRGB, pix)
    if pix.alpha:
        pix = pymupdf.Pixmap(pix, 0)
    save_webp(pix, FIG_DIR / f'{name}.webp', lossless=False)


def fit(pdf, data):
    """Least-squares line mapping PDF coordinates to data coordinates."""
    n = len(pdf)
    mp, md = sum(pdf) / n, sum(data) / n
    slope = sum((p - mp) * (d - md) for p, d in zip(pdf, data)) / sum((p - mp) ** 2 for p in pdf)
    return slope, md - slope * mp


def flatten(items, to_m):
    """A drawing's line and rectangle items as a list of points, in meters."""
    pts = []
    for it in items:
        if it[0] == 'l':
            for p in it[1:3]:
                q = to_m(p)
                if not pts or math.dist(pts[-1], q) > 1e-6:
                    pts.append(q)
        elif it[0] == 're':
            r = it[1]
            pts += [to_m(p) for p in (r.tl, r.tr, r.br, r.bl)]
        elif it[0] == 'qu':
            q = it[1]
            pts += [to_m(p) for p in (q.ul, q.ur, q.lr, q.ll)]
        elif it[0] == 'c':
            raise SystemExit('Fig. 2 paths now contain curves; extend flatten()')
    return pts


def export_ladder(doc):
    page, cap = find_figure(doc, 'Fig. 2.')
    drawings = [d for d in page.get_drawings() if d['rect'].y1 < cap.y0]
    spans = [s for b in page.get_text('dict')['blocks'] for l in b.get('lines', []) for s in l['spans']]

    # Panels are the white-filled plot areas, ordered (a) (b) / (c) (d).
    frames = sorted(
        (d['rect'] for d in drawings if d['type'] == 'f' and rgb(d.get('fill')) == (1.0, 1.0, 1.0) and d['rect'].width > 100),
        key=lambda r: (round(r.y0 / 20), r.x0),
    )
    if len(frames) != 4:
        raise SystemExit(f'expected 4 panels in Fig. 2, found {len(frames)}')
    panels = dict(zip('abcd', frames))

    def panel_of(rect):
        c = ((rect.x0 + rect.x1) / 2, (rect.y0 + rect.y1) / 2)
        for p, f in panels.items():
            if f.x0 - 2 <= c[0] <= f.x1 + 2 and f.y0 - 2 <= c[1] <= f.y1 + 2:
                return p

    # Axis maps from tick marks (short black segments just outside each frame) and their labels.
    maps = {}
    for p, f in panels.items():
        ticks_x, ticks_y = [], []
        for d in drawings:
            r = d['rect']
            if d.get('color') is None or rgb(d['color']) != BLACK or len(d['items']) != 1 or d['items'][0][0] != 'l':
                continue
            if r.width < 0.01 and 2 < r.height < 5 and abs(r.y0 - f.y1) < 0.5 and f.x0 <= r.x0 <= f.x1:
                ticks_x.append(r.x0)
            elif r.height < 0.01 and 2 < r.width < 5 and abs(r.x1 - f.x0) < 0.5 and f.y0 <= r.y0 <= f.y1:
                ticks_y.append(r.y0)

        def label_near(x, y):
            best = min(spans, key=lambda s: math.dist(((s['bbox'][0] + s['bbox'][2]) / 2, (s['bbox'][1] + s['bbox'][3]) / 2), (x, y)))
            return float(best['text'].replace('−', '-'))

        xs = [label_near(x, f.y1 + 6) for x in ticks_x]
        ys = [label_near(f.x0 - 10, y) for y in ticks_y]
        maps[p] = (*fit(ticks_x, xs), *fit(ticks_y, ys))

    out = {p: {'obstacles': [], 'dots': [], 'discs': [], 'start': None, 'primary': None, 'support': None} for p in panels}
    for d in drawings:
        p = panel_of(d['rect'])
        if not p:
            continue
        sx, bx, sy, by = maps[p]
        to_m = lambda pt: (sx * pt.x + bx, sy * pt.y + by)
        fill, stroke, r = rgb(d.get('fill')), rgb(d.get('color')), d['rect']
        center = to_m(pymupdf.Point((r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2))
        if fill == OBSTACLE:
            pts = flatten(d['items'], to_m)
            if len(pts) >= 3:
                out[p]['obstacles'].append(pts)
        elif fill == DISC:
            out[p]['discs'].append((center, flatten(d['items'], to_m)))
        elif fill == BLACK and 2 < r.width < 3:
            out[p]['dots'].append(center)
        elif fill == START:
            out[p]['start'] = center
        elif stroke == PRIMARY and d['type'] == 's':
            out[p]['primary'] = flatten(d['items'], to_m)
        elif stroke == SUPPORT and d['type'] == 's':
            out[p]['support'] = flatten(d['items'], to_m)

    a = out['a']
    # Panel (a) clips its discs; (b) draws them whole, so take each disc's extent there.
    landmarks = []
    for x, y in sorted(a['dots']):
        center, pts = min(out['b']['discs'], key=lambda d: math.dist(d[0], (x, y)))
        rx = (max(q[0] for q in pts) - min(q[0] for q in pts)) / 2
        ry = (max(q[1] for q in pts) - min(q[1] for q in pts)) / 2
        landmarks.append({'x': round(x, 1), 'y': round(y, 1), 'rx': round(rx, 1), 'ry': round(ry, 1)})

    r1 = lambda pts: [[round(x, 1), round(y, 1)] for x, y in pts]
    f = panels['a']
    sx, bx, sy, by = maps['a']
    goal = a['primary'][-1]  # the star's box center sits above the point; the path ends on it
    ladder = {
        '_source': 'Fig. 2 of the paper. Geometry and trajectories were read from the vector paths in the PDF; '
        'level, bound, length and terminal sigma are from the table beside the figure.',
        'units': 'm',
        'view': {'x0': round(sx * f.x0 + bx), 'x1': round(sx * f.x1 + bx), 'y0': round(sy * f.y1 + by), 'y1': round(sy * f.y0 + by)},
        'start': [round(v) for v in a['start']],
        'goal': [round(goal[0], 1), round(goal[1], 1)],
        'obstacles': [r1(o) for o in a['obstacles']],
        'landmarks': landmarks,
        'levels': [],
    }
    for p in 'abcd':
        level, bound, length, sigma, note = LEVELS[p]
        primary = out[p]['primary']
        drawn = sum(math.dist(primary[i], primary[i + 1]) for i in range(len(primary) - 1))
        if abs(drawn - length) > 5:
            print(f'warning: panel ({p}) primary is {drawn:.0f} m as drawn, {length} m in the table')
        ladder['levels'].append({'panel': p, 'level': level, 'bound': bound, 'length': length, 'sigma': sigma,
                                 'note': note, 'primary': r1(primary), 'support': r1(out[p]['support'])})
    DATA.parent.mkdir(parents=True, exist_ok=True)
    DATA.write_text(json.dumps(ladder, separators=(',', ':')))
    print(f'{DATA.relative_to(SITE)}  {DATA.stat().st_size} bytes')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    pdf = pymupdf.open(sys.argv[1])
    export_figures(pdf)
    export_ladder(pdf)
