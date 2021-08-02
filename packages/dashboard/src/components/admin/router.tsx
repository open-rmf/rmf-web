import React from 'react';
import { Redirect, Route, Switch, useRouteMatch } from 'react-router-dom';
import { AdminDrawer } from './drawer';
import { RoleListPage } from './role-list-page';
import { UserListPage } from './user-list-page';
import { UserProfilePage } from './user-profile-page';

export function AdminRouter(): JSX.Element {
  const match = useRouteMatch();

  return (
    <>
      <Switch>
        <Route path={`${match.path}/users/:user`}>
          <UserProfilePage />
        </Route>
        <Route exact path={`${match.path}/users`}>
          <UserListPage />
        </Route>
        <Route exact path={`${match.path}/roles`}>
          <RoleListPage />
        </Route>
        <Route>
          <Redirect to={`${match.path}/users`} />
        </Route>
      </Switch>
      <AdminDrawer />
    </>
  );
}
