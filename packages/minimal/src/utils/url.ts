export const BasePath =
  process.env.PUBLIC_URL === undefined || process.env.PUBLIC_URL === '/'
    ? ''
    : process.env.PUBLIC_URL;
export const DashboardRoute = BasePath === '' ? '/' : BasePath;
export const LoginRoute = `${BasePath}/login`;
