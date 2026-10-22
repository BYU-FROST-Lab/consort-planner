"""Convert the paper's plot PDFs into the site's figure images.

    python3 scripts/export_plots.py        (from website/; needs Ghostscript and Pillow)

Renders each PDF in PLOTS from code/figures/paper with Ghostscript, trims it to
a fixed white margin, and writes it to src/assets/figures/ as lossless WebP.
Each is a single 3.5 in panel, so the site can set them side by side.
Re-run it when the figures in code/figures/paper change.
"""

import shutil
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageOps

SITE = Path(__file__).resolve().parent.parent
SOURCE = SITE.parent / 'code' / 'figures' / 'paper'
OUT = SITE / 'src' / 'assets' / 'figures'
DPI = 300  # a 3.5 in panel comes out about 1050 px wide: 2x its on-page width
MARGIN = 8

PLOTS = {
    'fig3a-success': 'fig3a.pdf',
    'fig3b-length': 'fig3b.pdf',
    'fig4-runtime': 'fig4.pdf',
    'fig5a-ablation-success': 'fig5a.pdf',
    'fig5b-ablation-length': 'fig5b.pdf',
    'fig6-ablation-runtime': 'fig6.pdf',
}


def trim(img):
    """Crop to the non-white content plus MARGIN, so every panel has the same border."""
    box = ImageOps.invert(img.convert('L')).point(lambda v: 255 if v > 5 else 0).getbbox()
    if not box:
        return img
    x0, y0, x1, y1 = box
    return img.crop((max(x0 - MARGIN, 0), max(y0 - MARGIN, 0), min(x1 + MARGIN, img.width), min(y1 + MARGIN, img.height)))


def main():
    gs = shutil.which('gs')
    if not gs:
        raise SystemExit('Ghostscript (gs) not found; install it, e.g. `brew install ghostscript`')
    OUT.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        for name, pdf in PLOTS.items():
            png = Path(tmp) / f'{name}.png'
            subprocess.run([gs, '-q', '-dSAFER', '-dBATCH', '-dNOPAUSE', '-sDEVICE=png16m', f'-r{DPI}',
                            '-dTextAlphaBits=4', '-dGraphicsAlphaBits=4', f'-sOutputFile={png}', str(SOURCE / pdf)],
                           check=True)
            img = trim(Image.open(png).convert('RGB'))
            dest = OUT / f'{name}.webp'
            img.save(dest, 'WEBP', lossless=True, quality=100, method=6)
            print(f'{dest.relative_to(SITE)}  {img.width}x{img.height}  {dest.stat().st_size // 1024} KB')


if __name__ == '__main__':
    main()
