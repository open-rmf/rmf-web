import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import 'react-grid-layout/css/styles.css';
import './app.css';

import React from 'react';
import { Navigate, Route, Routes } from 'react-router-dom';

import { AppConfigContext, AuthenticatorContext, ResourcesContext } from './app-config';
import {
  AdminRouter,
  AppBase,
  AppEvents,
  ManagedWorkspace,
  PrivateRoute,
  RmfDashboard,
  SettingsContext,
  Workspace,
  WorkspaceState,
} from './components';
import { LoginPage } from './pages';
import KeycloakAuthenticator from './services/keycloak';
import {
  AdminRoute,
  CustomRoute1,
  CustomRoute2,
  DashboardRoute,
  LoginRoute,
  RobotsRoute,
  TasksRoute,
} from './utils/url';

const dashboardWorkspace: WorkspaceState = {
  layout: [{ i: 'map', x: 0, y: 0, w: 12, h: 12 }],
  windows: [{ key: 'map', appName: 'Map' }],
};

const robotsWorkspace: WorkspaceState = {
  layout: [
    { i: 'robots', x: 0, y: 0, w: 7, h: 4 },
    { i: 'map', x: 8, y: 0, w: 5, h: 8 },
    { i: 'doors', x: 0, y: 0, w: 7, h: 4 },
    { i: 'lifts', x: 0, y: 0, w: 7, h: 4 },
    { i: 'mutexGroups', x: 8, y: 0, w: 5, h: 4 },
  ],
  windows: [
    { key: 'robots', appName: 'Robots' },
    { key: 'map', appName: 'Map' },
    { key: 'doors', appName: 'Doors' },
    { key: 'lifts', appName: 'Lifts' },
    { key: 'mutexGroups', appName: 'Mutex Groups' },
  ],
};

const tasksWorkspace: WorkspaceState = {
  layout: [
    { i: 'tasks', x: 0, y: 0, w: 7, h: 12 },
    { i: 'map', x: 8, y: 0, w: 5, h: 12 },
  ],
  windows: [
    { key: 'tasks', appName: 'Tasks' },
    { key: 'map', appName: 'Map' },
  ],
};

export default function App() {
  return (
    <RmfDashboard
      apiServerUrl="http://localhost:8000"
      trajectoryServerUrl="http://localhost:8006"
      authenticator={
        new KeycloakAuthenticator({
          url: 'http://localhost:8080',
          realm: 'master',
          clientId: 'rmf-dashboard',
        })
      }
    />
  );
}

// export default function App(): JSX.Element | null {
//   const authenticator = React.useContext(AuthenticatorContext);
//   const [authInitialized, setAuthInitialized] = React.useState(!!authenticator.user);
//   const [user, setUser] = React.useState<string | null>(authenticator.user || null);

//   React.useEffect(() => {
//     let cancel = false;
//     const onUserChanged = (newUser: string | null) => {
//       setUser(newUser);
//       AppEvents.justLoggedIn.next(true);
//     };
//     authenticator.on('userChanged', onUserChanged);
//     (async () => {
//       await authenticator.init();
//       if (cancel) {
//         return;
//       }
//       setUser(authenticator.user || null);
//       setAuthInitialized(true);
//     })();
//     return () => {
//       cancel = true;
//       authenticator.off('userChanged', onUserChanged);
//     };
//   }, [authenticator]);

//   const appConfig = React.useContext(AppConfigContext);
//   const settings = React.useContext(SettingsContext);
//   const resources = appConfig.resources[settings.themeMode] || appConfig.resources.default;

//   const loginRedirect = React.useMemo(() => <Navigate to={LoginRoute} />, []);

//   return authInitialized ? (
//     <ResourcesContext.Provider value={resources}>
//       {user ? (
//         <RmfDashboard
//           apiServerUrl="http://localhost:8080"
//           trajectoryServerUrl="http://localhost:8081"
//         >
//           <AppBase>
//             <Routes>
//               <Route path={LoginRoute} element={<Navigate to={DashboardRoute} />} />

//               <Route
//                 path={DashboardRoute}
//                 element={
//                   <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
//                     <Workspace key="dashboard" state={dashboardWorkspace} />
//                   </PrivateRoute>
//                 }
//               />

//               <Route
//                 path={RobotsRoute}
//                 element={
//                   <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
//                     <Workspace key="robots" state={robotsWorkspace} />
//                   </PrivateRoute>
//                 }
//               />

//               <Route
//                 path={TasksRoute}
//                 element={
//                   <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
//                     <Workspace key="tasks" state={tasksWorkspace} />
//                   </PrivateRoute>
//                 }
//               />

//               {APP_CONFIG_ENABLE_CUSTOM_TABS && (
//                 <Route
//                   path={CustomRoute1}
//                   element={
//                     <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
//                       <ManagedWorkspace key="custom1" workspaceId="custom1" />
//                     </PrivateRoute>
//                   }
//                 />
//               )}

//               {APP_CONFIG_ENABLE_CUSTOM_TABS && (
//                 <Route
//                   path={CustomRoute2}
//                   element={
//                     <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
//                       <ManagedWorkspace key="custom2" workspaceId="custom2" />
//                     </PrivateRoute>
//                   }
//                 />
//               )}

//               {APP_CONFIG_ENABLE_ADMIN_TAB && (
//                 <Route
//                   path={AdminRoute}
//                   element={
//                     <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
//                       <AdminRouter />
//                     </PrivateRoute>
//                   }
//                 />
//               )}
//             </Routes>
//           </AppBase>
//         </RmfDashboard>
//       ) : (
//         <Routes>
//           <Route
//             path={LoginRoute}
//             element={
//               <LoginPage
//                 title={'Dashboard'}
//                 logo={resources.logos.header}
//                 onLoginClick={() =>
//                   authenticator.login(`${window.location.origin}${DashboardRoute}`)
//                 }
//               />
//             }
//           />
//           <Route path="*" element={<Navigate to={LoginRoute} />} />
//         </Routes>
//       )}
//     </ResourcesContext.Provider>
//   ) : null;
// }
