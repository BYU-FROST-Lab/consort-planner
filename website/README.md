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
| Figures 3–6 | `src/assets/figures/`, converted from their PDFs in `code/figures/paper` by `scripts/export_plots.py` |
| Figures 1, 7, and 8 | `src/assets/figures/`, cropped from the paper by `scripts/extract_figures.py` |
| Constraint ladder (Fig. 2) | `src/data/constraint-ladder.json`, built from the planner's Fig. 2 runs in `code/results/paper/ladder` by `scripts/export_ladder.py` |
| Table I | `src/components/ResultsTable.astro`, typed in from the paper |
| Footer links, favicons | `src/data/footer.ts` and `public/`, copied from the FROST Lab site; keep them in sync |

The constraint ladder replays Fig. 2's four plans as an interactive SVG. `scripts/export_ladder.py` reads each run's B-spline control points, evaluates the spline with a port of the planner's sampler, and refuses to write unless the primary's length matches `results.yaml` to a millimeter. Fig. 2's configs set `cont_unc_use_waypoints: true`, so the planner schedules exchanges (every 100 m) and weighs landmarks along the control polygon, not the spline. The replay does the same: it runs on that polygon's arc length, takes each exchange's weight from it with the planner's tapers (half weight at 300 m for communication, 100 m for landmarks), and draws the agents on the flown splines at the matching point. The tapers have no hard cutoff, so an exchange beyond 300 m still counts at reduced weight; those are drawn dashed. Re-run the script with `python3 scripts/export_ladder.py` if the runs change.

## Placeholders to replace

All of them live in `src/config.ts`, and `npm run build` warns about the first one until it's set:

- `arxivId`: enables the Paper and arXiv buttons and fills in the BibTeX
- `venue`: shown in orange under the title once set (hidden for now)
- `publication.booktitle`: once accepted, switches the BibTeX from arXiv `@misc` to `@inproceedings`

## Keeping the repo small

The site's committed assets are about 0.6 MB. To keep it that way:

- **Videos go on YouTube**, not in the repo: set `youtubeId`. The optional looping hero video (`heroVideo` in `src/config.ts`, file in `public/videos/`) should be short, muted, and under ~2 MB. Commit it once: every revision of a binary stays in git history.
- **Figures are lossless WebP** at about twice their on-page width (20–140 KB each). Pages serve them as committed, without re-encoding, so the plot text stays sharp.
- **Photos are lossy WebP** at quality 80. The HoloOcean render is 194 KB, and Astro builds the hero's responsive AVIF sizes from it at build time. Build output isn't committed.

To refresh the plots after `code/figures/paper` changes, run `python3 scripts/export_plots.py` (needs Ghostscript and Pillow). Figs. 1, 7, and 8 come from the paper itself; to refresh them from a new PDF (e.g. the camera-ready):

```bash
pip install pymupdf pillow
python scripts/extract_figures.py path/to/paper.pdf
```

## Analytics

Google Analytics is set up as on the lab site: the build reads the measurement ID from `PUBLIC_GA_ID`, and CI sets it from the repository variable of the same name. To use the lab's property, give this repo the same value as the lab site's variable:

```bash
gh variable set PUBLIC_GA_ID --repo BYU-FROST-Lab/consort-planner --body "$(gh variable get PUBLIC_GA_ID --repo BYU-FROST-Lab/byu-frost-lab.github.io)"
```

Without the variable, as in local builds, the page loads no analytics.

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
