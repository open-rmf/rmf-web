/**
 * Always starts with a '/'.
 */
export const BASE_PATH = (() => {
  if (!process.env.REACT_APP_BASE_PATH || process.env.REACT_APP_BASE_PATH === '') {
    return '/';
  }
  if (!process.env.REACT_APP_BASE_PATH.startsWith('/')) {
    return `/${process.env.REACT_APP_BASE_PATH}`;
  }
  return process.env.REACT_APP_BASE_PATH;
})();

export const LOGIN_ROUTE = `${BASE_PATH}/login`;
