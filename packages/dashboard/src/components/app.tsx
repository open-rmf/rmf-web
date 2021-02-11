import { createMuiTheme, ThemeProvider } from '@material-ui/core';
import React from 'react';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import 'typeface-roboto';
import appConfig from '../app-config';
import { DASHBOARD_ROUTE, LOGIN_ROUTE } from '../util/url';
import { AppBase } from './app-base';
import './app.css';
import { UserContext } from './auth/contexts';
import Login from './auth/login';
import PrivateRoute from './auth/private-route';
import { User } from './auth/user';
import Dashboard from './dashboard/dashboard';
import NotFoundPage from './error-pages/page-not-found';
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

  return authInitialized ? (
    <UserContext.Provider value={user}>
      <ThemeProvider theme={theme}>
        <BrowserRouter>
          <Switch>
            <Route exact path={LOGIN_ROUTE}>
              <Login />
            </Route>
            {/* we need this because we don't want to re-mount `AppIntrinsics` when just moving
          from one route to another, but we want to unmount it when going "outside" the app. */}
            <PrivateRoute exact path={appRoutes}>
              <AppIntrinsics>
                <Switch>
                  <PrivateRoute exact path={DASHBOARD_ROUTE}>
                    <Dashboard />
                  </PrivateRoute>
                </Switch>
              </AppIntrinsics>
            </PrivateRoute>
            <Route>
              <NotFoundPage />
            </Route>
          </Switch>
        </BrowserRouter>
      </ThemeProvider>
    </UserContext.Provider>
  ) : null;
}
