import type { IconName } from '../components/Icon.astro';

// The FROST Lab site's footer links (footerData in src/navigation.ts of the
// byu-frost-lab.github.io repo), with its site-relative links made absolute.
// Keep the two in sync.
const lab = 'https://frostlab.byu.edu';

export const footerColumns: { title: string; links: { text: string; href: string }[] }[] = [
  {
    title: 'FROST Lab Links',
    links: [
      { text: 'People', href: `${lab}/people/` },
      { text: 'Research', href: `${lab}/research/` },
      { text: 'Publications', href: `${lab}/publications/` },
      { text: 'Robots', href: `${lab}/robots/` },
      { text: 'Facilities', href: `${lab}/facilities/` },
      { text: 'Get Involved', href: `${lab}/get-involved/` },
    ],
  },
  {
    title: 'Related Webpages',
    links: [
      { text: 'BYU Homepage', href: 'https://byu.edu/' },
      { text: 'BYU Robotics REU', href: 'https://robotics.byu.edu/' },
      { text: 'BYU ECE', href: 'https://ece.byu.edu/' },
      { text: 'BYU Mech. Eng.', href: 'https://www.me.byu.edu/' },
    ],
  },
  {
    title: 'Social Media',
    links: [
      { text: 'LinkedIn', href: 'https://www.linkedin.com/company/byu-frost-lab/' },
      { text: 'GitHub', href: 'https://github.com/byu-frost-lab' },
      { text: 'YouTube', href: 'https://www.youtube.com/@byu-frost-lab' },
      { text: 'Twitter', href: 'https://x.com/byu_frost_lab' },
    ],
  },
  {
    title: 'Resources',
    links: [
      { text: 'Office of Belonging', href: 'https://belonging.byu.edu/' },
      { text: 'Student Development', href: 'https://sds.byu.edu' },
    ],
  },
];

export const socialLinks: { label: string; icon: IconName; href: string }[] = [
  { label: 'LinkedIn', icon: 'social-linkedin', href: 'https://www.linkedin.com/company/byu-frost-lab/' },
  { label: 'GitHub', icon: 'social-github', href: 'https://github.com/byu-frost-lab' },
  { label: 'YouTube', icon: 'social-youtube', href: 'https://www.youtube.com/@byu-frost-lab' },
  { label: 'X', icon: 'social-x', href: 'https://twitter.com/byu_frost_lab' },
];
