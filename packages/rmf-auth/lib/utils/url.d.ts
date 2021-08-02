/**
 * Normalizes a path so that it:
 *   * starts with a '/'
 *   * never ends with '/', unless it is '/'
 *   * does not have repeated forward slashes
 * @param pathOrUrl if an url is provided, returns only the normalized path portion of the url
 */
export declare function normalizePath(pathOrUrl: string): string;
export declare const BASE_PATH: string;
/**
 * Resolves a relative to the normalized full path.
 * @param path must be in a format accepted by `normalizePath`
 * @param basePath must be in a format accepted by `normalizePath`
 */
export declare function getFullPath(path: string, basePath?: string): string;
/**
 * Get a qualified url based on the path. The schema and hostname is obtained based on the
 * current `window.location`.
 */
export declare function getUrl(path: string): string;
export declare const LOGIN_ROUTE: string;
