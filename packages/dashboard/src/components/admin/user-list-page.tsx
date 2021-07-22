/* istanbul ignore file */

import React from 'react';
import { RmfIngressContext } from '../rmf-app';
import { usePageStyles } from './page-css';
import { UserListCard } from './user-list-card';

export function UserListPage(): JSX.Element | null {
  const classes = usePageStyles();
  const rmfIngress = React.useContext(RmfIngressContext);
  const adminApi = rmfIngress?.adminApi;

  if (!adminApi) return null;

  return (
    <div className={classes.pageRoot}>
      <UserListCard
        searchUsers={async (search, limit, offset) =>
          (await adminApi.getUsersAdminUsersGet(search, undefined, limit, offset)).data
        }
        deleteUser={async (user) => {
          await adminApi.deleteUserAdminUsersUsernameDelete(user);
        }}
        createUser={async (user) => {
          await adminApi.createUserAdminUsersPost({ username: user });
        }}
      />
    </div>
  );
}
