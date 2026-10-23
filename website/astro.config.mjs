// @ts-check
import { defineConfig } from 'astro/config';
import tailwindcss from '@tailwindcss/vite';

// ---------------------------------------------------------------------------
// DEPLOYMENT TARGET
// ---------------------------------------------------------------------------
// Built for GitHub Pages as a *project* site of the BYU-FROST-Lab org. The
// org site (the byu-frost-lab.github.io repo) has the custom domain
// frostlab.byu.edu, so this repo's site is served at
// https://frostlab.byu.edu/consort-planner/ with no DNS setup of its own.
// The path is the repo name: renaming the repo moves the site.
//
// Both values below must agree with where the site is served, or every asset
// 404s. Leave "Custom domain" empty in this repo's Pages settings so it
// inherits the org's domain. Publishing is off by default; see README.md.
// ---------------------------------------------------------------------------
export default defineConfig({
  site: 'https://frostlab.byu.edu',
  base: '/consort-planner',
  vite: { plugins: [tailwindcss()] },
});
