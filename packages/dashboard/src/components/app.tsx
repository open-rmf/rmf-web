import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import React from 'react';
import { NotFoundPage } from 'react-components';
import { BrowserRouter, Link, Redirect, Route, Switch, useLocation } from 'react-router-dom';
import { getUrl, LoginHOC, PrivateRouteHOC, User } from 'rmf-auth';
import 'typeface-roboto';
import appConfig from '../app-config';
import ResourceManager from '../managers/resource-manager';
import { DASHBOARD_ROUTE, LOGIN_ROUTE } from '../util/url';
import { AppBase } from './app-base';
import { AppConfigContext, ResourcesContext } from './app-contexts';
import './app.css';
import { AuthenticatorContext, UserContext } from './auth/contexts';
import Dashboard from './dashboard/dashboard';
import { RmfApp } from './rmf-app';

const theme = createMuiTheme({
  palette: {
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
});

function AppIntrinsics({ children }: React.PropsWithChildren<{}>): JSX.Element | null {
  return (
    <AppBase>
      <RmfApp>{children}</RmfApp>
    </AppBase>
  );
}

export default function App(): JSX.Element | null {
  const authenticator = appConfig.authenticator;
  const [authInitialized, setAuthInitialized] = React.useState(!!authenticator.user);
  const [user, setUser] = React.useState<User | null>(authenticator.user || null);
  const appRoutes = [DASHBOARD_ROUTE];

  React.useEffect(() => {
    if (user) {
      return;
    }
    const onUserChanged = (newUser: User | null) => setUser(newUser);
    authenticator.on('userChanged', onUserChanged);
    (async () => {
      await authenticator.init();
      setUser(authenticator.user || null);
      setAuthInitialized(true);
    })();
    return () => {
      authenticator.off('userChanged', onUserChanged);
    };
  }, [authenticator, user]);

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

  const PrivateRoute = PrivateRouteHOC(Route, Redirect, useLocation);
  const Login = LoginHOC(Redirect);

  return authInitialized && appReady ? (
    <AppConfigContext.Provider value={appConfig}>
      <ResourcesContext.Provider value={resourceManager.current}>
        <AuthenticatorContext.Provider value={authenticator}>
          <UserContext.Provider value={user}>
            <ThemeProvider theme={theme}>
              <BrowserRouter>
                <Switch>
                  <Route exact path={LOGIN_ROUTE}>
                    <Login
                      user={user}
                      title={'Dashboard'}
                      authenticator={authenticator}
                      successRedirectUri={getUrl(DASHBOARD_ROUTE)}
                    />
                  </Route>
                  {/* we need this because we don't want to re-mount `AppIntrinsics` when just moving
                  from one route to another, but we want to unmount it when going "outside" the app. */}
                  <PrivateRoute exact path={appRoutes} redirectPath={LOGIN_ROUTE} user={user}>
                    <AppIntrinsics>
                      <Switch>
                        <PrivateRoute
                          exact
                          path={DASHBOARD_ROUTE}
                          redirectPath={LOGIN_ROUTE}
                          user={user}
                        >
                          <Dashboard />
                        </PrivateRoute>
                      </Switch>
                    </AppIntrinsics>
                  </PrivateRoute>
                  <Route>
                    <NotFoundPage routeLinkComponent={<Link to={LOGIN_ROUTE}>Go to Login</Link>} />
                  </Route>
                </Switch>
              </BrowserRouter>
            </ThemeProvider>
          </UserContext.Provider>
        </AuthenticatorContext.Provider>
      </ResourcesContext.Provider>
    </AppConfigContext.Provider>
  ) : null;
}
