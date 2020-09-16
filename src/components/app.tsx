import React from 'react';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import 'typeface-roboto';
import appConfig from '../app-config';
import { LOGIN_ROUTE } from '../util/url';
import './app.css';
import { AuthenticatorContext, UserContext } from './auth/contexts';
import Login from './auth/login';
import PrivateRoute from './auth/private-route';
import { User } from './auth/user';
import Dashboard from './dashboard';
import NotFoundPage from './error-pages/page-not-found';
import { HotKeys } from 'react-hotkeys';

export default function App(): React.ReactElement {
  const [authInitialized, setAuthInitialized] = React.useState(false);
  const [user, setUser] = React.useState<User | null>(null);
  const authenticator = appConfig.authenticator;

  React.useEffect(() => {
    (async () => {
      authenticator.on('userChanged', (newUser) => setUser(newUser));
      await authenticator.init();
      setUser(authenticator.user || null);
      setAuthInitialized(true);
    })();
  }, [authenticator]);

  const keyMap = {
    SNAP_LEFT: '-',
    OPEN_COMMANDS: ['shift+space'],
    // SHOW_ALL_HOTKEYS: 'shift+?',
  };

  const handlers = {
    SNAP_LEFT: (event: any) => console.log('Move up hotkey called!'),
    OPEN_COMMANDS: (event: any) => console.log('Move up hotkey called!'),
    MOVE_UP: (event: any) => console.log('Move up hotkey called!'),
  };

  return authInitialized ? (
    <HotKeys keyMap={keyMap} handlers={handlers}>
      <AuthenticatorContext.Provider value={authenticator}>
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
      </AuthenticatorContext.Provider>
    </HotKeys>
  ) : (
    <></>
  );
}
