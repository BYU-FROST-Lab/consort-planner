"""Regenerate the site's figure images from the paper PDF.

    pip install pymupdf pillow
    python scripts/extract_figures.py path/to/paper.pdf

Writes src/assets/figures/fig*.webp, relative to website/: the paper's figures
as lossless WebP at about 2x their on-page width (fig7 is a photo, lossy).

Only needed when the figures change, e.g. for the camera-ready PDF. It finds
each figure by its caption ("Fig. N."), so update FIGURES if those change.
Fig. 2 is not an image on the site: scripts/export_ladder.py builds its data
from the planner's runs.
"""

import sys
from pathlib import Path

import pymupdf
from PIL import Image, ImageOps

SITE = Path(__file__).resolve().parent.parent
FIG_DIR = SITE / 'src' / 'assets' / 'figures'
DPI = 500

# (output name, caption label). Fig. 2 is drawn from the planner's data instead.
FIGURES = [
    ('fig1-escort-problem', 'Fig. 1.'),
    ('fig3-baselines', 'Fig. 3.'),
    ('fig4-runtime', 'Fig. 4.'),
    ('fig5-ablation', 'Fig. 5.'),
    ('fig6-ablation-runtime', 'Fig. 6.'),
    ('fig8-monte-carlo', 'Fig. 8.'),
]
PHOTO = ('fig7-holoocean', 'Fig. 7.')

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


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    pdf = pymupdf.open(sys.argv[1])
    export_figures(pdf)
