import { normalizePath } from 'rmf-auth';

export const BASE_PATH = normalizePath(process.env.PUBLIC_URL || '/');
export const DASHBOARD_ROUTE = BASE_PATH;
export const LOGIN_ROUTE = normalizePath(`${BASE_PATH}/login`);
export const BUILDING_ROUTE = normalizePath(`${BASE_PATH}/building`);
export const ROBOTS_ROUTE = normalizePath(`${BASE_PATH}/robots`);
export const TASKS_ROUTE = normalizePath(`${BASE_PATH}/tasks`);
export const LOGS_ROUTE = normalizePath(`${BASE_PATH}/logs`);

export const TAB_NAMES = ['Overview'];
