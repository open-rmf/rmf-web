import React, { useState } from 'react';
import { DASHBOARD_ROUTE, LOGIN_ROUTE, DEFAULT_ROUTE } from '../util/url';
import Dashboard from './dashboard';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import Login from './login/login';
import { UserContext } from '../app-contexts';
import { isEmpty } from 'lodash';
import NotFoundPage from './page-not-found';
import PrivateRoute from './privateRoute';
import 'typeface-roboto';

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
            isAuthenticated={!isEmpty(user)}
            exact={true}
            component={Dashboard}
            path={DASHBOARD_ROUTE}
          />
          <PrivateRoute
            redirectToLogin={true}
            isAuthenticated={!isEmpty(user)}
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
