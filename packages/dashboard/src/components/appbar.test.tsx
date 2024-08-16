import { Tab } from '@mui/material';
import { waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { beforeEach, describe, expect, it, vi } from 'vitest';

import { AuthenticatorProvider } from '../hooks/use-authenticator';
import { Resources, ResourcesProvider } from '../hooks/use-resources';
import { UserProfileProvider } from '../hooks/use-user-profile';
import { UserProfile } from '../services/authenticator';
import { StubAuthenticator } from '../services/stub-authenticator';
import { render } from '../utils/test-utils.test';
import { AppController, AppControllerContext } from './app-contexts';
import AppBar from './appbar';

function makeMockAppController(): AppController {
  return {
    updateSettings: vi.fn(),
    showAlert: vi.fn(),
    setExtraAppbarIcons: vi.fn(),
  };
}

const profile: UserProfile = {
  user: { username: 'test', is_admin: false, roles: [] },
  permissions: [],
};

const resources: Resources = {
  fleets: {},
  logos: {
    header: '/test-logo.png',
  },
};

describe('AppBar', () => {
  let appController: AppController;
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <AppControllerContext.Provider value={appController}>
        <ResourcesProvider value={resources}>
          <UserProfileProvider value={profile}>{props.children}</UserProfileProvider>
        </ResourcesProvider>
      </AppControllerContext.Provider>
    );
  };

  beforeEach(() => {
    appController = makeMockAppController();
  });

  it('renders with navigation bar', () => {
    const root = render(
      <Base>
        <AppBar
          tabs={[<Tab key="test" label="test" value="test" />]}
          tabValue="test"
          helpLink=""
          reportIssueLink=""
        />
      </Base>,
    );
    expect(root.getAllByRole('tablist').length > 0).toBeTruthy();
  });

  it('user button is shown when there is an authenticated user', () => {
    const root = render(
      <Base>
        <AppBar
          tabs={[<Tab key="test" label="test" value="test" />]}
          tabValue="test"
          helpLink=""
          reportIssueLink=""
        />
      </Base>,
    );
    expect(root.getByLabelText('user-btn')).toBeTruthy();
  });

  it('logout is triggered when logout button is clicked', async () => {
    const authenticator = new StubAuthenticator('test');
    const spy = vi.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);
    const root = render(
      <AuthenticatorProvider value={authenticator}>
        <Base>
          <AppBar
            tabs={[<Tab key="test" label="test" value="test" />]}
            tabValue="test"
            helpLink=""
            reportIssueLink=""
          />
        </Base>
      </AuthenticatorProvider>,
    );
    userEvent.click(root.getByLabelText('user-btn'));
    await expect(waitFor(() => root.getByText('Logout'))).resolves.not.toThrow();
    userEvent.click(root.getByText('Logout'));
    await expect(waitFor(() => expect(spy).toHaveBeenCalledTimes(1))).resolves.not.toThrow();
  });

  it('uses headerLogo from logo resources manager', async () => {
    const root = render(
      <Base>
        <AppBar
          tabs={[<Tab key="test" label="test" value="test" />]}
          tabValue="test"
          helpLink=""
          reportIssueLink=""
        />
      </Base>,
    );
    await expect(
      waitFor(() => {
        const q = root.container.querySelector('[src="/test-logo.png"][alt="logo"]');
        if (!q) throw new Error();
        return q;
      }),
    ).resolves.not.toThrow();
  });
});
