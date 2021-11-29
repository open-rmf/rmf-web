/* istanbul ignore file */

import { Typography } from '@mui/material';
import { User } from 'api-client';
import { AxiosError } from 'axios';
import React from 'react';
import { useAsync } from 'react-components';
import { useRouteMatch } from 'react-router';
import { RmfIngressContext } from '../rmf-app';
import { getApiErrorMessage } from '../utils';
import { ManageRolesCard } from './manage-roles-dialog';
import { adminPageClasses, AdminPageContainer } from './page-css';
import { UserProfileCard } from './user-profile';

export function UserProfilePage(): JSX.Element | null {
  const match = useRouteMatch<{ user: string }>();
  const userId: string | undefined = match.params.user;
  const safeAsync = useAsync();
  const { adminApi } = React.useContext(RmfIngressContext) || {};
  const [user, setUser] = React.useState<User | undefined>(undefined);
  const [notFound, setNotFound] = React.useState(false);

  const refresh = React.useCallback(() => {
    if (!adminApi || !userId) return;
    (async () => {
      try {
        setUser((await safeAsync(adminApi.getUserAdminUsersUsernameGet(userId))).data);
      } catch (e) {
        if ((e as AxiosError).response?.status !== 404) {
          throw new Error(getApiErrorMessage(e));
        }
        setNotFound(true);
      }
    })();
  }, [adminApi, safeAsync, userId]);

  React.useEffect(() => {
    refresh();
  }, [refresh]);

  return adminApi ? (
    <AdminPageContainer className={adminPageClasses.pageRoot}>
      {notFound ? (
        <Typography variant="h6" className={adminPageClasses.notFound}>
          404 Not Found
        </Typography>
      ) : (
        user && (
          <>
            <UserProfileCard
              user={user}
              makeAdmin={async (admin) => {
                try {
                  await adminApi.makeAdminAdminUsersUsernameMakeAdminPost(user.username, { admin });
                  refresh();
                } catch (e) {
                  throw new Error(getApiErrorMessage(e));
                }
              }}
            />
            <ManageRolesCard
              className={adminPageClasses.manageRoles}
              assignedRoles={user.roles}
              getAllRoles={async () => {
                try {
                  return (await adminApi.getRolesAdminRolesGet()).data;
                } catch (e) {
                  throw new Error(getApiErrorMessage(e));
                }
              }}
              saveRoles={async (roles) => {
                try {
                  await adminApi.setUserRolesAdminUsersUsernameRolesPut(
                    user.username,
                    roles.map((r) => ({
                      name: r,
                    })),
                  );
                  refresh();
                } catch (e) {
                  throw new Error(getApiErrorMessage(e));
                }
              }}
            />
          </>
        )
      )}
    </AdminPageContainer>
  ) : null;
}
