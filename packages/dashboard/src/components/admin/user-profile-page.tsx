/* istanbul ignore file */

import { makeStyles, Paper, Typography } from '@material-ui/core';
import { User } from 'api-client';
import { AxiosError } from 'axios';
import React from 'react';
import { useAsync } from 'react-components';
import { useRouteMatch } from 'react-router';
import { RmfIngressContext } from '../rmf-app';
import { getApiErrorMessage } from '../utils';
import { ManageRolesCard } from './manage-roles-dialog';
import { usePageStyles } from './page-css';
import { UserProfileCard } from './user-profile';

const useStyles = makeStyles((theme) => ({
  manageRoles: {
    marginTop: theme.spacing(4),
  },
}));

export function UserProfilePage(): JSX.Element | null {
  const pageClasses = usePageStyles();
  const classes = useStyles();
  const match = useRouteMatch<{ user: string }>();
  const user: string | undefined = match.params.user;
  const safeAsync = useAsync();
  const { adminApi } = React.useContext(RmfIngressContext) || {};
  const [profile, setProfile] = React.useState<User | undefined>(undefined);
  const [notFound, setNotFound] = React.useState(false);

  React.useEffect(() => {
    if (!adminApi || !user) return;
    (async () => {
      try {
        setProfile((await safeAsync(adminApi.getUserAdminUsersUsernameGet(user))).data);
      } catch (e) {
        if ((e as AxiosError).response?.status !== 404) {
          throw new Error(getApiErrorMessage(e));
        }
        setNotFound(true);
      }
    })();
  }, [adminApi, user, safeAsync]);

  return adminApi && profile ? (
    <div className={pageClasses.pageRoot}>
      {notFound ? (
        <Paper>
          <Typography>404 Not Found</Typography>
        </Paper>
      ) : (
        <>
          <UserProfileCard profile={profile} />
          <ManageRolesCard
            className={classes.manageRoles}
            assignedRoles={profile.roles}
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
                  roles.map((r) => ({
                    name: r,
                  })),
                  profile.username,
                );
              } catch (e) {
                throw new Error(getApiErrorMessage(e));
              }
            }}
          />
        </>
      )}
    </div>
  ) : null;
}
