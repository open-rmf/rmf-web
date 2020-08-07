import React, { useState } from 'react';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import 'typeface-roboto';
import { UserContext } from '../app-contexts';
import { DASHBOARD_ROUTE, DEFAULT_ROUTE, LOGIN_ROUTE } from '../util/url';
import Dashboard from './dashboard';
import Login from './login/login';
import NotFoundPage from './page-not-found';
import PrivateRoute from './privateRoute';
import './app.css';

export default function App() {
  // TODO: replace with the data of the authentication service.
  // eslint-disable-next-line
  const [user, setUser] = useState({ user: 'Admin' });
  return (
    <UserContext.Provider value={user}>
      <BrowserRouter>
        <Switch>
          <Route exact={true} path={LOGIN_ROUTE} component={Login} />
          <PrivateRoute
            redirectToLogin={true}
            isAuthenticated={!user}
            exact={true}
            component={Dashboard}
            path={DASHBOARD_ROUTE}
          />
          <PrivateRoute
            redirectToLogin={true}
            isAuthenticated={!user}
            exact={true}
            path={DEFAULT_ROUTE}
            component={Dashboard}
          />
          {/* TODO: When the authentication logic is implemented this should be the default route  */}
          <Route path="*" component={NotFoundPage} />
        </Switch>
      </BrowserRouter>
    </UserContext.Provider>
  );
}
