/**
 * Everything that names the paper, its authors, or a link lives here.
 * Fields marked TODO are placeholders until the paper is public; the build
 * prints a warning for each one still unset (see src/lib/placeholders.ts).
 */

export const project = {
  name: 'CONSORT',
  title:
    'CONSORT: Cooperative Navigation with Support-Optimized Routing and Trajectories for GPS-Denied Escort Missions',
  /** One sentence for search results and link previews. */
  description:
    'CONSORT routes support agents so that a primary agent reaches its goal in minimum time with its localization uncertainty under a terminal bound, in GPS-denied environments.',
  /**
   * TODO: the venue once accepted, e.g. 'ICRA 2027'. Shown in orange under
   * the title. Set to undefined to hide the line.
   */
  venue: 'Under review' as string | undefined,
};

export interface Author {
  name: string;
  url?: string;
  /** 1-based indices into `affiliations`. */
  affiliations: number[];
  /** Marks equal contribution with an asterisk. */
  equal?: boolean;
}

/**
 * In paper order. Links come from the FROST Lab people pages; TODO: add one for
 * each author who has a personal page.
 */
export const authors: Author[] = [
  { name: 'Tony Zhang', affiliations: [1, 2] },
  { name: 'Ryan Bartholomew', url: 'https://sites.google.com/view/ryan-m-bartholomew', affiliations: [2] },
  { name: 'Kalliyan Velasco', affiliations: [2] },
  { name: 'Joshua Mangelson', affiliations: [2] },
  { name: 'Brady Moon', url: 'https://bradymoon.com', affiliations: [2] },
];

export const affiliations = ['University of Utah', 'Brigham Young University'];

/**
 * TODO before publishing: the preprint's arXiv ID, e.g. '2610.01234'. The
 * Paper and arXiv buttons and the BibTeX all come from it.
 */
export const arxivId: string | undefined = undefined;

/**
 * TODO: the YouTube video ID (the part after `watch?v=`). It fills the video
 * section under the hero, which shows a "coming soon" frame until then, and
 * enables the Video button.
 */
export const youtubeId: string | undefined = undefined;

/**
 * Optional looping hero video, e.g. 'videos/hero.mp4' for a file in
 * public/videos/. It replaces the HoloOcean still behind the title. Keep it
 * short, muted, and under ~2 MB: it is committed to the repo.
 */
export const heroVideo: string | undefined = undefined;

/** BibTeX fields. Until `booktitle` is set, the entry is arXiv's @misc format. */
export const publication = {
  /** Year of the arXiv posting, or of the proceedings once `booktitle` is set. */
  year: 2026,
  /** TODO after acceptance, e.g. 'IEEE International Conference on Robotics and Automation (ICRA)'. */
  booktitle: undefined as string | undefined,
};

export const repoUrl = 'https://github.com/BYU-FROST-Lab/consort-planner';

/**
 * The hero buttons. The paper's footnote promises the Julia implementation, the
 * 50 benchmark scenarios, and simulation videos here: Code, Benchmark, and Video
 * cover them. A button without a link shows as "soon".
 */
export const links = {
  paper: arxivId && `https://arxiv.org/pdf/${arxivId}`,
  arxiv: arxivId && `https://arxiv.org/abs/${arxivId}`,
  code: repoUrl,
  /** TODO: the scenarios' folder in the repo once the code is in, e.g. `${repoUrl}/tree/main/scenarios`. */
  benchmark: undefined as string | undefined,
  video: youtubeId && `https://youtu.be/${youtubeId}`,
};

export const lab = {
  name: 'FROST Lab',
  fullName: 'Field Robotic Systems Lab',
  url: 'https://frostlab.byu.edu',
};

/** Google Analytics measurement ID ('G-XXXXXXXXXX'). Undefined means no analytics. */
export const googleAnalyticsId: string | undefined = undefined;
