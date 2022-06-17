import React from 'react';
import { RmfAppContext } from '../rmf-app';
import { getApiErrorMessage } from '../utils';
import { adminPageClasses, AdminPageContainer } from './page-css';
import { UserListCard } from './user-list-card';

export function UserListPage(): JSX.Element | null {
  const rmfIngress = React.useContext(RmfAppContext);
  const adminApi = rmfIngress?.adminApi;

  if (!adminApi) return null;

  return (
    <AdminPageContainer className={adminPageClasses.pageRoot}>
      <UserListCard
        searchUsers={async (search, limit, offset) => {
          try {
            return (await adminApi.getUsersAdminUsersGet(search, undefined, limit, offset)).data;
          } catch (e) {
            throw new Error(getApiErrorMessage(e));
          }
        }}
        deleteUser={async (user) => {
          try {
            await adminApi.deleteUserAdminUsersUsernameDelete(user);
          } catch (e) {
            throw new Error(getApiErrorMessage(e));
          }
        }}
        createUser={async (user) => {
          try {
            await adminApi.createUserAdminUsersPost({ username: user });
          } catch (e) {
            throw new Error(getApiErrorMessage(e));
          }
        }}
      />
    </AdminPageContainer>
  );
}
