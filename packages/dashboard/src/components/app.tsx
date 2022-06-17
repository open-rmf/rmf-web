import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import DesignModeIcon from '@mui/icons-material/AutoFixNormal';
import { IconButton, useTheme } from '@mui/material';
import React from 'react';
import 'react-grid-layout/css/styles.css';
import { useRouteMatch } from 'react-router';
import { Redirect, Route, Switch } from 'react-router-dom';
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
import { RobotPage } from './robots';
import { tasksWorkspace } from './tasks-workspace';
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
   * If resource loading gets too long we should add a loading screen.
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

  const loginRedirect = React.useMemo(() => <Redirect to={LoginRoute} />, []);

  const customRoute1 = useRouteMatch(CustomRoute1);
  const customRoute2 = useRouteMatch(CustomRoute2);
  const isCustomRoute = customRoute1 || customRoute2;

  const [designMode, setDesignMode] = React.useState(false);
  const theme = useTheme();

  const extraToolbarItems = React.useMemo(() => {
    if (isCustomRoute) {
      return (
        <IconButton
          color="inherit"
          sx={{ opacity: designMode ? undefined : theme.palette.action.disabledOpacity }}
          onClick={() => setDesignMode((prev) => !prev)}
        >
          <DesignModeIcon />
        </IconButton>
      );
    }
    return null;
  }, [isCustomRoute, theme, designMode]);

  return authInitialized && appReady ? (
    <ResourcesContext.Provider value={resourceManager.current}>
      {user ? (
        <RmfApp>
          <AppBase extraToolbarItems={extraToolbarItems}>
            <Switch>
              <Route exact path={LoginRoute}>
                <Redirect to={DashboardRoute} />
              </Route>
              <PrivateRoute
                exact
                path={DashboardRoute}
                unauthorizedComponent={loginRedirect}
                user={user}
              >
                <Workspace state={dashboardWorkspace} />
              </PrivateRoute>
              <PrivateRoute
                exact
                path={RobotsRoute}
                unauthorizedComponent={loginRedirect}
                user={user}
              >
                <RobotPage />
              </PrivateRoute>
              <PrivateRoute
                exact
                path={TasksRoute}
                unauthorizedComponent={loginRedirect}
                user={user}
              >
                <Workspace state={tasksWorkspace} />
              </PrivateRoute>
              <PrivateRoute unauthorizedComponent={loginRedirect} user={user} {...customRoute1}>
                <ManagedWorkspace workspaceId="custom1" designMode={designMode} />
              </PrivateRoute>
              <PrivateRoute unauthorizedComponent={loginRedirect} user={user} {...customRoute2}>
                <ManagedWorkspace workspaceId="custom2" designMode={designMode} />
              </PrivateRoute>
              <PrivateRoute path={AdminRoute} unauthorizedComponent={loginRedirect} user={user}>
                <AdminRouter />
              </PrivateRoute>
              <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
                <Redirect to={DashboardRoute} />
              </PrivateRoute>
            </Switch>
          </AppBase>
        </RmfApp>
      ) : (
        <Switch>
          <Route exact path={LoginRoute}>
            <LoginPage
              title={'Dashboard'}
              logo="assets/ros-health.png"
              onLoginClick={() => authenticator.login(`${window.location.origin}${DashboardRoute}`)}
            />
          </Route>
          <Route>
            <Redirect to={LoginRoute} />
          </Route>
        </Switch>
      )}
    </ResourcesContext.Provider>
  ) : null;
}
