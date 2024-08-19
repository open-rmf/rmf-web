import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import './app.css';

import React from 'react';
import { Navigate, Route, Routes } from 'react-router-dom';

import { appConfig } from './app-config';
import {
  AppBase,
  AppEvents,
  PrivateRoute,
  RmfDashboard,
  SettingsContext,
  StaticWorkspace,
  Workspace,
  WorkspaceState,
} from './components';
import { MicroAppManifest } from './components/micro-app';
import doorsApp from './micro-apps/doors-app';
import liftsApp from './micro-apps/lifts-app';
import createMapApp from './micro-apps/map-app';
import robotMutexGroupsApp from './micro-apps/robot-mutex-groups-app';
import robotsApp from './micro-apps/robots-app';
import tasksApp from './micro-apps/tasks-app';
import { LoginPage } from './pages';
import KeycloakAuthenticator from './services/keycloak';
import StubAuthenticator from './services/stub-authenticator';
import {
  AdminRoute,
  CustomRoute1,
  CustomRoute2,
  DashboardRoute,
  LoginRoute,
  RobotsRoute,
  TasksRoute,
} from './utils/url';

const mapApp = createMapApp({
  attributionPrefix: appConfig.attributionPrefix,
  defaultMapLevel: appConfig.defaultMapLevel,
  defaultRobotZoom: appConfig.defaultRobotZoom,
  defaultZoom: appConfig.defaultZoom,
});

const homeWorkspace: WorkspaceState = {
  windows: {
    map: {
      layout: { x: 0, y: 0, w: 12, h: 6 },
      component: mapApp.Component,
    },
  },
};

const robotsWorkspace: WorkspaceState = {
  windows: {
    robots: {
      layout: { x: 0, y: 0, w: 7, h: 4 },
      component: robotsApp.Component,
    },
    map: { layout: { x: 8, y: 0, w: 5, h: 8 }, component: mapApp.Component },
    doors: { layout: { x: 0, y: 0, w: 7, h: 4 }, component: doorsApp.Component },
    lifts: { layout: { x: 0, y: 0, w: 7, h: 4 }, component: liftsApp.Component },
    mutexGroups: { layout: { x: 8, y: 0, w: 5, h: 4 }, component: robotMutexGroupsApp.Component },
  },
};

const tasksWorkspace: WorkspaceState = {
  windows: {
    tasks: { layout: { x: 0, y: 0, w: 7, h: 8 }, component: tasksApp.Component },
    map: { layout: { x: 8, y: 0, w: 5, h: 8 }, component: mapApp.Component },
  },
};

export default function App() {
  return (
    <RmfDashboard
      apiServerUrl="http://localhost:8000"
      trajectoryServerUrl="http://localhost:8006"
      authenticator={new StubAuthenticator()}
      helpLink={appConfig.helpLink}
      reportIssueLink={appConfig.reportIssue}
      resources={appConfig.resources.default}
      tasks={{
        allowedTasks: appConfig.allowedTasks,
        pickupZones: appConfig.pickupZones,
        cartIds: appConfig.cartIds,
      }}
      tabs={[
        {
          name: 'Map',
          route: '/',
          element: <StaticWorkspace key="Map" initialState={homeWorkspace} />,
        },
        {
          name: 'Robots',
          route: '/robots',
          element: <StaticWorkspace key="Robots" initialState={robotsWorkspace} />,
        },
        {
          name: 'Tasks',
          route: '/tasks',
          element: <StaticWorkspace key="Tasks" initialState={tasksWorkspace} />,
        },
      ]}
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
