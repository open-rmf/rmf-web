import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import React from 'react';
import { NotFoundPage } from 'react-components';
import appConfig from '../app-config';
import ResourceManager from '../managers/resource-manager';
import { AppBase } from './app-base';
import { AppConfigContext, ResourcesContext, TrajectorySocketContext } from './app-contexts';
import './app.css';
import { AuthenticatorContext } from './auth/contexts';
import { RmfApp } from './rmf-app';
import { Route, Redirect, useLocation, Switch } from 'react-router';
import { BrowserRouter, Link } from 'react-router-dom';
import { getUrl, LoginHOC, PrivateRouteHOC } from 'rmf-auth';
import { DASHBOARD_ROUTE, LOGIN_ROUTE } from '../util/url';
import Dashboard from './dashboard/dashboard';

const PrivateRoute = PrivateRouteHOC(Route, Redirect, useLocation);
const Login = LoginHOC(Redirect);

const theme = createMuiTheme({
  palette: {
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
});

//  think you should have react-components be able to take in various props which controls what data to render. In the app itself, have some build time configuration which "hardcodes" those values passed as props to the components.
// One way to do it is to generate an app-config.ts as part of the build process, you could use handlebars or other templating engines to do it, or keep it simple and just have the config as a typescript source directly. An environment variable at build time would choose which app-config.ts to use, if not provided, the default can be used, or the build can fail.
// Ideally we should avoid downloading a json config file at runtime, or use dynamic import() as those solutions requires separate http requests.

export default function App(): JSX.Element | null {
  const [authInitialized, setAuthInitialized] = React.useState(!!appConfig.authenticator.user);
  const [authenticator, setAuthenticator] = React.useState(appConfig.authenticator);
  const [user, setUser] = React.useState<string | null>(appConfig.authenticator.user || null);
  const [ws, setWs] = React.useState<WebSocket>(new WebSocket(appConfig.trajServerUrl));
  const appRoutes = [DASHBOARD_ROUTE];

  const onTokenExpired = () => setAuthenticator(appConfig.authenticator);
  authenticator.on('tokenRefresh', onTokenExpired);

  React.useEffect(() => {
    setWs(new WebSocket(appConfig.trajServerUrl));
  }, [setWs]);

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

  return authInitialized && appReady ? (
    <AppConfigContext.Provider value={appConfig}>
      <ResourcesContext.Provider value={resourceManager.current}>
        <AuthenticatorContext.Provider value={authenticator}>
          <TrajectorySocketContext.Provider value={ws}>
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
                  {user ? (
                    <PrivateRoute exact path={appRoutes} redirectPath={LOGIN_ROUTE} user={user}>
                      <RmfApp>
                        <AppBase>
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
                        </AppBase>
                      </RmfApp>
                    </PrivateRoute>
                  ) : (
                    <Route>
                      <NotFoundPage
                        routeLinkComponent={<Link to={LOGIN_ROUTE}>Go to Login</Link>}
                      />
                    </Route>
                  )}
                </Switch>
              </BrowserRouter>
            </ThemeProvider>
          </TrajectorySocketContext.Provider>
        </AuthenticatorContext.Provider>
      </ResourcesContext.Provider>
    </AppConfigContext.Provider>
  ) : null;
}
