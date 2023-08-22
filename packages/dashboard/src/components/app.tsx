import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import React from 'react';
import 'react-grid-layout/css/styles.css';
import { Route, Routes, Navigate } from 'react-router-dom';
import { LoginPage, PrivateRoute } from 'rmf-auth';
import appConfig from '../app-config';
import ResourceManager from '../managers/resource-manager';
import {
  AdminRoute,
  CustomRoute1,
  CustomRoute2,
  DashboardRoute,
  LoginRoute,
  RobotsRoute,
  TasksRoute,
} from '../util/url';
import { AdminRouter } from './admin';
import { AppBase } from './app-base';
import { ResourcesContext } from './app-contexts';
import './app.css';
import { dashboardWorkspace } from './dashboard';
import { RmfApp } from './rmf-app';
import { robotsWorkspace } from './robots/robots-workspace';
import { tasksWorkspace } from './tasks/tasks-workspace';
import { ManagedWorkspace, Workspace } from './workspace';

export default function App(): JSX.Element | null {
  const authenticator = appConfig.authenticator;
  const [authInitialized, setAuthInitialized] = React.useState(!!appConfig.authenticator.user);
  const [user, setUser] = React.useState<string | null>(authenticator.user || null);

  React.useEffect(() => {
    let cancel = false;
    const onUserChanged = (newUser: string | null) => setUser(newUser);
    authenticator.on('userChanged', onUserChanged);
    (async () => {
      await authenticator.init();
      if (cancel) {
        return;
      }
      setUser(authenticator.user || null);
      setAuthInitialized(true);
    })();
    return () => {
      cancel = true;
      authenticator.off('userChanged', onUserChanged);
    };
  }, [authenticator]);

  const resourceManager = React.useRef<ResourceManager | undefined>(undefined);
  const [appReady, setAppReady] = React.useState(false);

  /**
   * TODO: If resource loading gets too long we should add a loading screen.
   */
  React.useEffect(() => {
    (async () => {
      const appResources = await appConfig.appResourcesFactory();
      if (!appResources) {
        setAppReady(true);
      } else {
        resourceManager.current = appResources;
        setAppReady(true);
      }
    })();
  }, []);

  const loginRedirect = React.useMemo(() => <Navigate to={LoginRoute} />, []);

  return authInitialized && appReady ? (
    <ResourcesContext.Provider value={resourceManager.current}>
      {user ? (
        <RmfApp>
          <AppBase>
            <Routes>
              <Route path={LoginRoute} element={<Navigate to={DashboardRoute} />} />

              <Route
                path={DashboardRoute}
                element={
                  <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
                    <Workspace key="dashboard" state={dashboardWorkspace} />
                  </PrivateRoute>
                }
              />

              <Route
                path={RobotsRoute}
                element={
                  <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
                    <Workspace key="robots" state={robotsWorkspace} />
                  </PrivateRoute>
                }
              />

              <Route
                path={TasksRoute}
                element={
                  <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
                    <Workspace key="tasks" state={tasksWorkspace} />
                  </PrivateRoute>
                }
              />

              <Route
                path={CustomRoute1}
                element={
                  <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
                    <ManagedWorkspace key="custom1" workspaceId="custom1" />
                  </PrivateRoute>
                }
              />

              <Route
                path={CustomRoute2}
                element={
                  <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
                    <ManagedWorkspace key="custom2" workspaceId="custom2" />
                  </PrivateRoute>
                }
              />

              <Route
                path={AdminRoute}
                element={
                  <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
                    <AdminRouter />
                  </PrivateRoute>
                }
              />
            </Routes>
          </AppBase>
        </RmfApp>
      ) : (
        <Routes>
          <Route
            path={LoginRoute}
            element={
              <LoginPage
                title={'Dashboard'}
                logo="assets/defaultLogo.png"
                onLoginClick={() =>
                  authenticator.login(`${window.location.origin}${DashboardRoute}`)
                }
              />
            }
          />
          <Route path="*" element={<Navigate to={LoginRoute} />} />
        </Routes>
      )}
    </ResourcesContext.Provider>
  ) : null;
}
