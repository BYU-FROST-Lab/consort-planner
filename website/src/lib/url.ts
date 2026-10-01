const base = import.meta.env.BASE_URL.replace(/\/+$/, '');

/** Prefix a site-relative path with the configured base, e.g. url('favicon.svg'). */
export function url(path = ''): string {
  return `${base}/${path.replace(/^\/+/, '')}`;
}
