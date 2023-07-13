export const BasePath =
  process.env.PUBLIC_URL === undefined || process.env.PUBLIC_URL === '/'
    ? ''
    : process.env.PUBLIC_URL;
/**
 * Add /* after /admin to avoid the warning
 * <Routes> (or called `useRoutes()`) at "/admin"
 *  (under <Route path="/admin">)
 *  but the parent route path has no trailing "*".
 * This means if you navigate deeper,
 * the parent won't match anymore and therefore the child routes will never render.
 */
export const DashboardRoute = BasePath === '' ? '/' : BasePath;
export const LoginRoute = `${BasePath}/login`;
export const TasksRoute = `${BasePath}/tasks`;
export const RobotsRoute = `${BasePath}/robots`;
export const AdminRoute = `${BasePath}/admin/*`;
export const CustomRoute1 = `${BasePath}/custom1`;
export const CustomRoute2 = `${BasePath}/custom2`;
