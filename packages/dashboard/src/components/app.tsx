import React from 'react';
import { BrowserRouter, Route, Switch } from 'react-router-dom';
import 'typeface-roboto';
import { BASE_PATH, LOGIN_ROUTE } from '../util/url';
import { AppContextProvider } from './app-context-provider';
import './app.css';
import Login from './auth/login';
import PrivateRoute from './auth/private-route';
import Dashboard from './dashboard';
import NotFoundPage from './error-pages/page-not-found';
import { RmfContextProvider } from './rmf-contexts';

export default function App(): JSX.Element {
  return (
    <AppContextProvider>
      <RmfContextProvider
        doorStates={doorStates}
        liftStates={liftStates}
        dispenserStates={dispenserStates}
      >
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
      </RmfContextProvider>
    </AppContextProvider>
  );
}
