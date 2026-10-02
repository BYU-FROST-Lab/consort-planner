# CONSORT website

The project page for CONSORT. It's a static [Astro](https://astro.build) site styled with Tailwind CSS, built for GitHub Pages at <https://frostlab.byu.edu/consort-planner/>.

**Not published.** The deploy job is off until you switch it on. See [Publishing](#publishing).

## Run it locally

Needs Node 22.12 or newer.

```bash
cd website
npm install
npm run dev
```

Then open <http://localhost:4321/consort-planner/>.

## Where the content comes from

| Content | Source |
|---|---|
| Title, authors, venue, links, BibTeX fields | `src/config.ts` |
| Section text | `src/pages/index.astro` |
| Figures 1 and 3–8 | `src/assets/figures/`, cropped from the paper by `scripts/extract_figures.py` |
| Constraint ladder (Fig. 2) | `src/data/constraint-ladder.json`, read from the figure's vector paths by the same script |
| Table I | `src/components/ResultsTable.astro`, typed in from the paper |
| Footer links, favicons | `src/data/footer.ts` and `public/`, copied from the FROST Lab site; keep them in sync |

The constraint ladder redraws Fig. 2 as an interactive SVG. Its obstacles, landmarks, and trajectories come from the PDF, and each level's primary length matches the paper's table to within a meter. The replay's landmark readings, relay links, and communication-range ring are computed from the paper's model and experimental setup (exchanges every 100 m of arc; logistic tapers at 100 m for landmarks and 300 m for communication), not exported from the planner.

## Placeholders to replace

All of them live in `src/config.ts`, and `npm run build` warns about the first two until they're set:

- `arxivId`: enables the Paper and arXiv buttons and fills in the BibTeX
- `youtubeId`: replaces the "Video coming soon" frame under the hero and enables the Video button
- `venue`: shown in orange under the title, currently "Under review"
- `publication.booktitle`: once accepted, switches the BibTeX from arXiv `@misc` to `@inproceedings`

## Keeping the repo small

The site's committed assets are about 0.6 MB. To keep it that way:

- **Videos go on YouTube**, not in the repo: set `youtubeId`. The optional looping hero video (`heroVideo` in `src/config.ts`, file in `public/videos/`) should be short, muted, and under ~2 MB. Commit it once: every revision of a binary stays in git history.
- **Figures are lossless WebP** at about twice their on-page width (20–140 KB each). Pages serve them as committed, without re-encoding, so the plot text stays sharp.
- **Photos are lossy WebP** at quality 80. The HoloOcean render is 194 KB, and Astro builds the hero's responsive AVIF sizes from it at build time. Build output isn't committed.

To refresh the figures from a new PDF (e.g. the camera-ready), run the extraction script. It needs Python with PyMuPDF and Pillow. Fig. 2's table is an image in the PDF, so the script holds its numbers in `LEVELS`; update those by hand if they change.

```bash
pip install pymupdf pillow
python scripts/extract_figures.py path/to/paper.pdf
```

## Checks

```bash
npm run check    # type-check pages and components
npm run build    # build to dist/
npm run preview  # serve dist/ locally
```

GitHub Actions runs `check` and `build` on pull requests and pushes to `main` that touch `website/` (see `.github/workflows/website.yml`).

## Publishing

1. Fill in the placeholders above, and check that the build no longer warns about them.
2. Make the repository public, or the Code button and the links into the repo will 404 for visitors.
3. In the repository's **Settings → Pages**, set **Source** to **GitHub Actions**. Leave **Custom domain** empty: the site inherits frostlab.byu.edu from the org site, as described in `astro.config.mjs`.
4. Switch on the deploy job and run it:

   ```bash
   gh variable set PUBLISH_WEBSITE --body true --repo BYU-FROST-Lab/consort-planner
   gh workflow run website.yml --repo BYU-FROST-Lab/consort-planner --ref main
   ```

After that, every push to `main` that touches `website/` redeploys the site. To take it offline again, delete the variable and unpublish in the Pages settings.
