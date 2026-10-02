import { arxivId, authors, project, publication } from '../config';

// Braces keep BibTeX styles from lowercasing the acronyms.
const title = project.title.replace('CONSORT', '{CONSORT}').replace('GPS', '{GPS}');
const authorList = authors.map((a) => a.name).join(' and ');
const firstSurname = authors[0]?.name.split(' ').at(-1)?.toLowerCase() ?? 'anon';
const key = `${firstSurname}${publication.year}consort`;

/**
 * The page's citation. arXiv's own @misc format until `publication.booktitle`
 * is set in config.ts, then @inproceedings.
 */
export function bibtex(): string {
  if (publication.booktitle) {
    return `@inproceedings{${key},
  title     = {${title}},
  author    = {${authorList}},
  booktitle = {${publication.booktitle}},
  year      = {${publication.year}}
}`;
  }
  const eprint = arxivId ?? 'XXXX.XXXXX';
  return `@misc{${key},
  title         = {${title}},
  author        = {${authorList}},
  year          = {${publication.year}},
  eprint        = {${eprint}},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/${eprint}}
}`;
}
