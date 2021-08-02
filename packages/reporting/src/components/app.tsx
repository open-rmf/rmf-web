import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import React from 'react';
import { NotFoundPage } from 'react-components';
import { BrowserRouter, Link, Redirect, Route, Switch } from 'react-router-dom';
import { LoginPage, PrivateRoute } from 'rmf-auth';
import appConfig from '../app-config';
import { DASHBOARD_ROUTE, LOGIN_ROUTE } from '../util/url';
import { AppConfigContext } from './app-contexts';
import { AuthenticatorContext, UserContext } from './auth-contexts';
import Dashboard from './dashboard';

const theme = createMuiTheme({
  palette: {
    primary: {
      main: '#44497a',
      dark: '#323558',
      light: '#565d99',
    },
  },
});

export default function App(): JSX.Element | null {
  const authenticator = appConfig.authenticator;
  const [authInitialized, setAuthInitialized] = React.useState(!!authenticator.user);
  const [user, setUser] = React.useState<string | null>(authenticator.user || null);
  const appRoutes = [DASHBOARD_ROUTE];

  React.useEffect(() => {
    if (user) {
      return;
    }
    const onUserChanged = (newUser: string | null) => setUser(newUser);
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

  const loginRedirect = React.useMemo(() => <Redirect to={LOGIN_ROUTE} />, []);

  return authInitialized ? (
    <AppConfigContext.Provider value={appConfig}>
      <AuthenticatorContext.Provider value={authenticator}>
        <UserContext.Provider value={user}>
          <ThemeProvider theme={theme}>
            <BrowserRouter>
              <Switch>
                <Route exact path={LOGIN_ROUTE}>
                  <LoginPage
                    title={'Reporting'}
                    logo="assets/ros-health.png"
                    onLoginClick={() =>
                      authenticator.login(`${window.location.origin}${DASHBOARD_ROUTE}`)
                    }
                  />
                </Route>
                <PrivateRoute
                  exact
                  path={appRoutes}
                  unauthorizedComponent={loginRedirect}
                  user={user}
                >
                  <Switch>
                    <PrivateRoute
                      exact
                      path={DASHBOARD_ROUTE}
                      unauthorizedComponent={loginRedirect}
                      user={user}
                    >
                      <Dashboard />
                    </PrivateRoute>
                  </Switch>
                </PrivateRoute>
                <Route>
                  <NotFoundPage routeLinkComponent={<Link to={LOGIN_ROUTE}>Go to Login</Link>} />
                </Route>
              </Switch>
            </BrowserRouter>
          </ThemeProvider>
        </UserContext.Provider>
      </AuthenticatorContext.Provider>
    </AppConfigContext.Provider>
  ) : null;
}
