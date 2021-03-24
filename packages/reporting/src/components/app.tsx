import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import React from 'react';
import { NotFoundPage } from 'react-components';
import { BrowserRouter, Link, Route, Switch } from 'react-router-dom';
import { User } from 'rmf-auth';
import 'typeface-roboto';
import appConfig from '../app-config';
import { DASHBOARD_ROUTE, LOGIN_ROUTE } from '../util/url';
import { AppConfigContext } from './app-contexts';
import { AuthenticatorContext, UserContext } from './auth-contexts';
import Login from './auth/login';
import { PrivateRoute } from './auth/private-route';
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

  return authInitialized ? (
    <AppConfigContext.Provider value={appConfig}>
      <AuthenticatorContext.Provider value={authenticator}>
        <UserContext.Provider value={user}>
          <ThemeProvider theme={theme}>
            <BrowserRouter>
              <Switch>
                <Route exact path={LOGIN_ROUTE}>
                  <Login />
                </Route>
                <PrivateRoute exact path={appRoutes}>
                  <Switch>
                    <PrivateRoute exact path={DASHBOARD_ROUTE}>
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
