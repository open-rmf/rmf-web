export const BASE_PATH =
  process.env.PUBLIC_URL === undefined || process.env.PUBLIC_URL === '/'
    ? ''
    : process.env.PUBLIC_URL;
export const DASHBOARD_ROUTE = BASE_PATH === '' ? '/' : BASE_PATH;
export const LOGIN_ROUTE = `${BASE_PATH}/login`;
