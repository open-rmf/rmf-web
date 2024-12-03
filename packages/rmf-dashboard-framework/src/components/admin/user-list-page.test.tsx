import { render as render_, waitFor } from '@testing-library/react';
import React from 'react';
import { MemoryRouter } from 'react-router';
import { describe, expect, it } from 'vitest';

import { AppControllerProvider, RmfApiProvider } from '../../hooks';
import { RmfApi } from '../../services';
import { makeMockAppController, MockRmfApi, TestProviders } from '../../utils/test-utils.test';
import { UserListPage } from './user-list-page';

const render = (ui: React.ReactNode) =>
  render_(<AppControllerProvider value={makeMockAppController()}>{ui}</AppControllerProvider>);

describe('UserListPage', () => {
  const Base = (props: React.PropsWithChildren<{}>) => {
    const rmfApi = React.useMemo<RmfApi>(() => {
      const mockRmfApi = new MockRmfApi();
      // mock out some api calls so they never resolves
      mockRmfApi.adminApi.getUsersAdminUsersGet = () => new Promise(() => {});
      mockRmfApi.adminApi.deleteUserAdminUsersUsernameDelete = () => new Promise(() => {});
      mockRmfApi.adminApi.createUserAdminUsersPost = () => new Promise(() => {});
      return mockRmfApi;
    }, []);
    return (
      <TestProviders>
        <RmfApiProvider value={rmfApi}>{props.children}</RmfApiProvider>
      </TestProviders>
    );
  };

  it('renders user list page', async () => {
    await expect(
      waitFor(() =>
        render(
          <Base>
            <MemoryRouter>
              <UserListPage />
            </MemoryRouter>
          </Base>,
        ),
      ),
    ).resolves.not.toThrow();
  });
});
