import React from 'react';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import 'typeface-roboto';
import makeAppConfig, { AppConfig } from '../app-config';
import { LOGIN_ROUTE } from '../util/url';
import './app.css';
import AuthContext from './auth/context';
import Login from './auth/login';
import PrivateRoute from './auth/private-route';
import Dashboard from './dashboard';
import NotFoundPage from './page-not-found';

export default function App() {
  const [appConfig, setAppConfig] = React.useState<AppConfig | null>(null);
  React.useEffect(() => {
    (async () => {
      setAppConfig(await makeAppConfig());
    })();
  }, []);

  return (
    appConfig && (
      <AuthContext.Provider value={appConfig.authenticator}>
        <BrowserRouter>
          <Switch>
            <Route exact={true} path={LOGIN_ROUTE} component={Login} />
            <PrivateRoute exact={true} path={'/'}>
              <Dashboard appConfig={appConfig} />
            </PrivateRoute>
            <Route component={NotFoundPage} />
          </Switch>
        </BrowserRouter>
      </AuthContext.Provider>
    )
  );
}
