/* istanbul ignore file */

import { makeStyles, Typography } from '@material-ui/core';
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
  notFound: {
    marginTop: '50%',
    textAlign: 'center',
  },
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

  const refresh = React.useCallback(() => {
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
  }, [adminApi, safeAsync, user]);

  React.useEffect(() => {
    refresh();
  }, [refresh]);

  return adminApi ? (
    <div className={pageClasses.pageRoot}>
      {notFound ? (
        <Typography variant="h6" className={classes.notFound}>
          404 Not Found
        </Typography>
      ) : (
        profile && (
          <>
            <UserProfileCard
              profile={profile}
              makeAdmin={async (admin) => {
                try {
                  await adminApi.makeAdminAdminUsersUsernameMakeAdminPost(
                    { admin },
                    profile.username,
                  );
                } catch (e) {
                  throw new Error(getApiErrorMessage(e));
                }
              }}
            />
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
                  refresh();
                } catch (e) {
                  throw new Error(getApiErrorMessage(e));
                }
              }}
            />
          </>
        )
      )}
    </div>
  ) : null;
}
