/* istanbul ignore file */

import React from 'react';
import { RmfIngressContext } from '../rmf-app';
import { usePageStyles } from './page-css';
import { RoleListCard } from './role-list-card';

export function RoleListPage(): JSX.Element | null {
  const classes = usePageStyles();
  const rmfIngress = React.useContext(RmfIngressContext);
  const adminApi = rmfIngress?.adminApi;

  if (!adminApi) return null;

  return (
    <div className={classes.pageRoot}>
      <RoleListCard
        getRoles={async () => (await adminApi.getRolesAdminRolesGet()).data}
        createRole={async (role) => {
          await adminApi.createRoleAdminRolesPost({ name: role });
        }}
        getPermissions={async (role) =>
          (await adminApi.getRolePermissionsAdminRolesRolePermissionsGet(role)).data
        }
        savePermission={async (role, permission) => {
          await adminApi.addRolePermissionAdminRolesRolePermissionsPost(permission, role);
        }}
      />
    </div>
  );
}
