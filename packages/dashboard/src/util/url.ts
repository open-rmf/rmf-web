import { normalizePath } from 'rmf-auth';

export const BASE_PATH = normalizePath(process.env.PUBLIC_URL || '/');
export const DASHBOARD_ROUTE = BASE_PATH;
export const LOGIN_ROUTE = normalizePath(`${BASE_PATH}/login`);

export const TAB_NAMES = ['Building'];
