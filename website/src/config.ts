/**
 * Everything that names the paper, its authors, or a link lives here.
 * Fields marked TODO are placeholders until the paper is public.
 */

export const project = {
  name: 'CONSORT',
  title:
    'CONSORT: Uncertainty-Constrained Multi-Robot Planning for Cooperative Localization in GPS-Denied Environments',
  /** One sentence for search results and link previews. */
  description:
    'CONSORT routes support agents so that a primary agent reaches its goal in minimum time with its localization uncertainty under a terminal bound, in GPS-denied environments.',
  /**
   * TODO: the venue once accepted, e.g. 'ICRA 2027'. Shown in orange under
   * the title; undefined hides the line.
   */
  venue: undefined as string | undefined,
};

export interface Author {
  name: string;
  url: string;
  /** 1-based indices into `affiliations`. */
  affiliations: number[];
}

/** In paper order. */
export const authors: Author[] = [
  { name: 'Tony Zhang', url: 'https://www.linkedin.com/in/tony-hu-zhang', affiliations: [1, 2] },
  { name: 'Ryan Bartholomew', url: 'https://sites.google.com/view/ryan-m-bartholomew', affiliations: [1] },
  { name: 'Kalliyan Velasco', url: 'https://frostlab.byu.edu/people/kalliyan-velasco/', affiliations: [1] },
  { name: 'Joshua Mangelson', url: 'https://frostlab.byu.edu/people/josh-mangelson/', affiliations: [1] },
  { name: 'Brady Moon', url: 'https://frostlab.byu.edu/people/brady-moon/', affiliations: [1] },
];

export const affiliations = ['Brigham Young University', 'University of Utah'];

/**
 * TODO before publishing: the preprint's arXiv ID, e.g. '2610.01234'. The
 * Paper and arXiv buttons and the BibTeX all come from it.
 */
export const arxivId: string | undefined = undefined;

/** The YouTube video ID (the part after `youtu.be/`), embedded under the hero and behind the Video button. */
export const youtubeId = 'prarYmi_3y4';

/** BibTeX fields. Until `booktitle` is set, the entry is arXiv's @misc format. */
export const publication = {
  /** Year of the arXiv posting, or of the proceedings once `booktitle` is set. */
  year: 2026,
  /** TODO after acceptance, e.g. 'IEEE International Conference on Robotics and Automation (ICRA)'. */
  booktitle: undefined as string | undefined,
};

export const repoUrl = 'https://github.com/BYU-FROST-Lab/consort-planner';

/**
 * The hero buttons. A button without a link shows as "soon". The benchmark
 * scenarios live in the same repo as the code, so the Code button covers them.
 */
export const links = {
  paper: arxivId && `https://arxiv.org/pdf/${arxivId}`,
  arxiv: arxivId && `https://arxiv.org/abs/${arxivId}`,
  code: repoUrl,
  video: `https://youtu.be/${youtubeId}`,
};

export const lab = {
  name: 'FROST Lab',
  url: 'https://frostlab.byu.edu',
};
