import { waitFor } from '@testing-library/react';
import React from 'react';
import { describe, expect, it } from 'vitest';

import { RmfApiProvider } from '../../hooks';
import { RmfApi } from '../../services';
import { MockRmfApi, render, TestProviders } from '../../utils/test-utils.test';
import { RoleListPage } from './role-list-page';

describe('Role List Page', () => {
  const Base = (props: React.PropsWithChildren<{}>) => {
    const rmfApi = React.useMemo<RmfApi>(() => {
      const mockRmfApi = new MockRmfApi();
      // mock out some api calls so they never resolves
      mockRmfApi.adminApi.getRolesAdminRolesGet = () => new Promise(() => {});
      mockRmfApi.adminApi.createRoleAdminRolesPost = () => new Promise(() => {});
      mockRmfApi.adminApi.deleteRoleAdminRolesRoleDelete = () => new Promise(() => {});
      mockRmfApi.adminApi.getRolePermissionsAdminRolesRolePermissionsGet = () =>
        new Promise(() => {});
      mockRmfApi.adminApi.addRolePermissionAdminRolesRolePermissionsPost = () =>
        new Promise(() => {});
      mockRmfApi.adminApi.removeRolePermissionAdminRolesRolePermissionsRemovePost = () =>
        new Promise(() => {});
      return mockRmfApi;
    }, []);
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders role list page', async () => {
    await expect(
      waitFor(() =>
        render(
          <Base>
            <RoleListPage />
          </Base>,
        ),
      ),
    ).resolves.not.toThrow();
  });
});
