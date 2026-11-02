# CONSORT website

## Run it locally

Needs Node 22.12 or newer.

```bash
cd website
npm install
npm run dev
```

Then open <http://localhost:4321/consort-planner/>. `npm run check` type-checks the pages and `npm run build` builds to `dist/`. GitHub Actions runs both on pull requests and pushes to `main` that touch `website/` (see `.github/workflows/website.yml`).

## Placeholders to replace

All of them live in `src/config.ts`:

- `project.venue`: shown in orange under the title once set (hidden for now)
- `publication.booktitle`: once accepted, switches the BibTeX from arXiv `@misc` to `@inproceedings`

