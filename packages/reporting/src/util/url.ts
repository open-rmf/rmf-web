import { BASE_PATH, normalizePath } from 'rmf-auth';

export const DASHBOARD_ROUTE = BASE_PATH;
export const LOGIN_ROUTE = normalizePath(`${BASE_PATH}/login`);
