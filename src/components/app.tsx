import React, { useState } from 'react';
import { AppConfig } from '../app-config';
import { DASHBOARD_ROUTE, LOGIN_ROUTE, REGISTER_ROUTE, DEFAULT_ROUTE } from '../util/url';
import Dashboard from './dashboard';
import { BrowserRouter, Route, Switch, Redirect } from 'react-router-dom';
import RegisterForm from './login/registerForm';
import Login from './login/login';
import { UserContext } from '../app-contexts';
import { isEmpty } from 'lodash';
import NotFoundPage from './page-not-found';
export interface AppProps {
  appConfig: AppConfig;
}

export default function App(props: AppProps) {
  // TODO: replace with the data of the authentication service.
  // eslint-disable-next-line
  const [user, setUser] = useState({ user: 'Admin' });
  return (
    <UserContext.Provider value={user}>
      <BrowserRouter>
        {isEmpty(user) && <Redirect to={LOGIN_ROUTE} />}
        <Switch>
          <Route exact={true} path={LOGIN_ROUTE} component={Login} />
          <Route exact={true} path={REGISTER_ROUTE} component={RegisterForm} />
          <Route redirectToLogin={true} exact={true} path={DASHBOARD_ROUTE}>
            {!isEmpty(user) && <Dashboard appConfig={props.appConfig}></Dashboard>}
          </Route>
          <Route exact={true} path={DEFAULT_ROUTE}>
            <Dashboard appConfig={props.appConfig}></Dashboard>
          </Route>
          {/* TODO: When the authentication logic is implemented this should be the default route  */}
          <Route path="*" component={NotFoundPage} />
        </Switch>
      </BrowserRouter>
    </UserContext.Provider>
  );
}
