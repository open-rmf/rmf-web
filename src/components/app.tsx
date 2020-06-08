import React from 'react';
import { AppConfig } from '../app-config';
import { DASHBOARD_ROUTE, LOGIN_ROUTE, REGISTER_ROUTE } from '../util/url';
import Dashboard from './dashboard';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import LoginForm from './login/loginForm';
import RegisterForm from './login/registerForm';
export interface AppProps {
  appConfig: AppConfig;
}

export default function App(props: AppProps) {
  return (
    <BrowserRouter>
      <Switch>
        <Route redirectToLogin={false} exact={true} path={DASHBOARD_ROUTE}>
          <Dashboard appConfig={props.appConfig}></Dashboard>
        </Route>
        <Route redirectToLogin={false} exact={true} path={LOGIN_ROUTE} component={LoginForm} />
        <Route redirectToLogin={false} path={REGISTER_ROUTE} component={RegisterForm} />
      </Switch>
    </BrowserRouter>
  );
}
