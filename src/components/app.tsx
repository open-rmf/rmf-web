import React from 'react';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import 'typeface-roboto';
import appConfig from '../app-config';
import { UserContext } from '../app-contexts';
import { LOGIN_ROUTE } from '../util/url';
import './app.css';
import Login from './auth/login';
import PrivateRoute from './auth/private-route';
import { User } from './auth/user';
import Dashboard from './dashboard';
import NotFoundPage from './page-not-found';

export default function App(): React.ReactElement {
  const [authInitialized, setAuthInitialized] = React.useState(false);
  const [user, setUser] = React.useState<User | null>(null);
  const authenticator = appConfig.authenticator;

  React.useEffect(() => {
    (async () => {
      authenticator.on('userChanged', newUser => setUser(newUser));
      await authenticator.init();
      setUser(authenticator.user || null);
      setAuthInitialized(true);
    })();
  }, [authenticator]);

  return authInitialized ? (
    <UserContext.Provider value={user}>
      <BrowserRouter>
        <Switch>
          <Route exact={true} path={LOGIN_ROUTE}>
            <Login />
          </Route>
          <PrivateRoute exact={true} path={'/'}>
            <Dashboard />
          </PrivateRoute>
          <Route component={NotFoundPage} />
        </Switch>
      </BrowserRouter>
    </UserContext.Provider>
  ) : (
    <></>
  );
}
