import React from 'react';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import 'typeface-roboto';
import { BASE_PATH, LOGIN_ROUTE } from '../util/url';
import { AppBase } from './app-contexts';
import './app.css';
import Login from './auth/login';
import PrivateRoute from './auth/private-route';
import Dashboard from './dashboard';
import NotFoundPage from './error-pages/page-not-found';
import { RmfApp } from './rmf-app';

export default function App(): JSX.Element {
  return (
    <AppBase>
      <RmfApp>
        <BrowserRouter>
          <Switch>
            <Route exact={true} path={LOGIN_ROUTE}>
              <Login />
            </Route>
            <PrivateRoute exact={true} path={BASE_PATH}>
              <Dashboard />
            </PrivateRoute>
            <Route component={NotFoundPage} />
          </Switch>
        </BrowserRouter>
      </RmfApp>
    </AppBase>
  );
}
