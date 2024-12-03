import { render as render_, waitFor } from '@testing-library/react';
import React from 'react';
import { describe, expect, it } from 'vitest';

import { AppControllerProvider, RmfApiProvider } from '../../hooks';
import { RmfApi } from '../../services';
import { makeMockAppController, MockRmfApi, TestProviders } from '../../utils/test-utils.test';
import { UserProfilePage } from './user-profile-page';

const render = (ui: React.ReactNode) =>
  render_(<AppControllerProvider value={makeMockAppController()}>{ui}</AppControllerProvider>);

describe('UserProfilePage', () => {
  const Base = (props: React.PropsWithChildren<{}>) => {
    const rmfApi = React.useMemo<RmfApi>(() => {
      const mockRmfApi = new MockRmfApi();
      // mock out some api calls so they never resolves
      mockRmfApi.adminApi.getUserAdminUsersUsernameGet = () => new Promise(() => {});
      mockRmfApi.adminApi.makeAdminAdminUsersUsernameMakeAdminPost = () => new Promise(() => {});
      mockRmfApi.adminApi.getRolesAdminRolesGet = () => new Promise(() => {});
      mockRmfApi.adminApi.setUserRolesAdminUsersUsernameRolesPut = () => new Promise(() => {});
      return mockRmfApi;
    }, []);
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders user profile page', async () => {
    await expect(
      waitFor(() =>
        render(
          <Base>
            <UserProfilePage />
          </Base>,
        ),
      ),
    ).resolves.not.toThrow();
  });
});
