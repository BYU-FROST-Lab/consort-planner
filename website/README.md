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

Then open <http://localhost:4321/consort-planner/>. `npm run check` type-checks the pages and `npm run build` builds to `dist/`. GitHub Actions runs both on pull requests and pushes to `main` that touch `website/` (see `.github/workflows/website.yml`).

## Where the content comes from

| Content | Source |
|---|---|
| Title, authors, venue, links, BibTeX fields | `src/config.ts` |
| Section text | `src/pages/index.astro` |
| Figures | `src/assets/figures/`: Figs. 3–6 rendered from their PDFs in `code/figures/paper`, Figs. 1, 7, and 8 cropped from the paper |
| Constraint ladder (Fig. 2) | `src/data/constraint-ladder.json`, exported from the planner's Fig. 2 runs in `code/results/paper/ladder` |
| Table I | `src/components/ResultsTable.astro`, typed in from the paper |
| Footer links, favicons | `src/data/footer.ts` and `public/`, copied from the FROST Lab site; keep them in sync |

The constraint ladder replays the planner's runs rather than redrawing the figure. Fig. 2's configs set `cont_unc_use_waypoints: true`, so the planner schedules its exchanges (every 100 m) and weighs landmarks along each agent's waypoint path. The replay runs on that path's arc length and draws the agents on the flown splines at the matching point. The communication weight halves at 300 m but has no hard cutoff, so an exchange beyond that range still counts at reduced weight; those are drawn dashed.

Figures are lossless WebP at about twice their on-page width, served as committed so the plot text stays sharp. Videos go on YouTube, not in the repo.

## Placeholders to replace

All of them live in `src/config.ts`:

- `arxivId`: enables the Paper and arXiv buttons and fills in the BibTeX
- `project.venue`: shown in orange under the title once set (hidden for now)
- `publication.booktitle`: once accepted, switches the BibTeX from arXiv `@misc` to `@inproceedings`

## Analytics

Google Analytics is set up as on the lab site: the build reads the measurement ID from `PUBLIC_GA_ID`, and CI sets it from the repository variable of the same name. To use the lab's property, give this repo the same value as the lab site's variable:

```bash
gh variable set PUBLIC_GA_ID --repo BYU-FROST-Lab/consort-planner --body "$(gh variable get PUBLIC_GA_ID --repo BYU-FROST-Lab/byu-frost-lab.github.io)"
```

Without the variable, as in local builds, the page loads no analytics.

## Publishing

1. Fill in the placeholders above.
2. Make the repository public, or the Code button and the links into the repo will 404 for visitors.
3. In the repository's **Settings → Pages**, set **Source** to **GitHub Actions**. Leave **Custom domain** empty: the site inherits frostlab.byu.edu from the org site, as described in `astro.config.mjs`.
4. Switch on the deploy job and run it:

   ```bash
   gh variable set PUBLISH_WEBSITE --body true --repo BYU-FROST-Lab/consort-planner
   gh workflow run website.yml --repo BYU-FROST-Lab/consort-planner --ref main
   ```

After that, every push to `main` that touches `website/` redeploys the site. To take it offline again, delete the variable and unpublish in the Pages settings.
