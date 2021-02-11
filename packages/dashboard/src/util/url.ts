/**
 * Normalizes a path so that it:
 *   * starts with a '/'
 *   * never ends with '/', unless it is '/'
 *   * does not have repeated forward slashes
 * @param path should only contain the path part of the url, i.e. no schema, hostname, search, fragment etc.
 * e.g.
 *
 *   "/foo/bar" ok
 *
 *   "http://example.com/foo/bar" bad
 *
 *   "/foo/bar?query=baz" bad
 */
export function normalizePath(path: string) {
  while (path.indexOf('//') !== -1) {
    path = path.replace(/\/\//g, '/');
  }
  if (!path.startsWith('/')) {
    path = `/${path}`;
  }
  if (path !== '/' && path.endsWith('/')) {
    path = path.slice(0, path.length - 1);
  }
  return path;
}

export const BASE_PATH = (() => {
  let path = process.env.PUBLIC_URL || '/';
  if (path.startsWith('http')) {
    path = new URL(path).pathname;
  }
  return normalizePath(path);
})();

/**
 * Resolves a relative to the normalized full path.
 * @param path must be in a format accepted by `normalizePath`
 * @param basePath must be in a format accepted by `normalizePath`
 */
export function getFullPath(path: string, basePath = BASE_PATH): string {
  return normalizePath(`${basePath}/${path}`);
}

/**
 * Get a qualified url based on the path. The schema and hostname is obtained based on the
 * current `window.location`.
 */
export function getUrl(path: string): string {
  return `${window.location.origin}${normalizePath(path)}`;
}

export const DASHBOARD_ROUTE = BASE_PATH;
export const LOGIN_ROUTE = normalizePath(`${BASE_PATH}/login`);
