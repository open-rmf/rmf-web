/**
 * Add /* after /admin to avoid the warning
 * <Routes> (or called `useRoutes()`) at "/admin"
 *  (under <Route path="/admin">)
 *  but the parent route path has no trailing "*".
 * This means if you navigate deeper,
 * the parent won't match anymore and therefore the child routes will never render.
 */
export const DashboardRoute = import.meta.env.BASE_URL;
export const LoginRoute = `${import.meta.env.BASE_URL}login`;
export const TasksRoute = `${import.meta.env.BASE_URL}tasks`;
export const RobotsRoute = `${import.meta.env.BASE_URL}robots`;
export const AdminRoute = `${import.meta.env.BASE_URL}admin/*`;
export const CustomRoute1 = `${import.meta.env.BASE_URL}custom1`;
export const CustomRoute2 = `${import.meta.env.BASE_URL}custom2`;
