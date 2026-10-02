import { arxivId, authors, links, youtubeId } from '../config';

/** Placeholders in src/config.ts that must be filled in before publishing. */
export function unsetPlaceholders(): string[] {
  const todo: string[] = [];
  if (authors.some((a) => /^(first|second|third) author$/i.test(a.name))) todo.push('authors');
  if (!arxivId) todo.push('arxivId (Paper and arXiv buttons, BibTeX)');
  if (!youtubeId) todo.push('youtubeId (Video button and section)');
  if (!links.benchmark) todo.push('links.benchmark (Benchmark button)');
  return todo;
}

let warned = false;

/** Logs the unset placeholders once per build. */
export function warnPlaceholders(): void {
  if (warned) return;
  warned = true;
  const todo = unsetPlaceholders();
  if (todo.length) {
    console.warn(`[consort] Placeholders still set in src/config.ts:\n  - ${todo.join('\n  - ')}`);
  }
}
