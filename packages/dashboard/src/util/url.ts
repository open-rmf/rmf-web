export const BASE_PATH =
  !process.env.REACT_APP_BASE_PATH || process.env.REACT_APP_BASE_PATH === '/'
    ? ''
    : process.env.REACT_APP_BASE_PATH;

export const DASHBOARD_ROUTE = `${BASE_PATH}/`;

export const LOGIN_ROUTE = `${BASE_PATH}/login`;
