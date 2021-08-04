/* istanbul ignore file */

import React from 'react';
import { RmfIngressContext } from '../rmf-app';
import { getApiErrorMessage } from '../utils';
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
          try {
            await adminApi.createRoleAdminRolesPost({ name: role });
          } catch (e) {
            throw new Error(getApiErrorMessage(e));
          }
        }}
        deleteRole={async (role) => {
          try {
            await adminApi.deleteRoleAdminRolesRoleDelete(role);
          } catch (e) {
            throw new Error(getApiErrorMessage(e));
          }
        }}
        getPermissions={async (role) => {
          try {
            return (await adminApi.getRolePermissionsAdminRolesRolePermissionsGet(role)).data;
          } catch (e) {
            throw new Error(getApiErrorMessage(e));
          }
        }}
        savePermission={async (role, permission) => {
          try {
            await adminApi.addRolePermissionAdminRolesRolePermissionsPost(permission, role);
          } catch (e) {
            throw new Error(getApiErrorMessage(e));
          }
        }}
        removePermission={async (role, permission) => {
          try {
            await adminApi.deleteRolePermissionAdminRolesRolePermissionsDelete(permission, role);
          } catch (e) {
            throw new Error(getApiErrorMessage(e));
          }
        }}
      />
    </div>
  );
}
