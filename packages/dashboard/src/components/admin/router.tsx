import React from 'react';
import { BrowserRouter, Redirect, Route, Switch } from 'react-router-dom';
import { AdminRoute } from '../../util/url';
import { AdminDrawer } from './drawer';
import { RoleListPage } from './role-list-page';
import { UserListPage } from './user-list-page';

export function AdminRouter(): JSX.Element {
  return (
    <BrowserRouter basename={AdminRoute}>
      <Switch>
        <Route exact path="/users">
          <UserListPage />
        </Route>
        <Route exact path="/roles">
          <RoleListPage />
        </Route>
        <Route>
          <Redirect to="/users" />
        </Route>
      </Switch>
      <AdminDrawer />
    </BrowserRouter>
  );
}
