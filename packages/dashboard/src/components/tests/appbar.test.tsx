import { waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { StubAuthenticator, UserProfile, UserProfileContext } from 'rmf-auth';
import { AuthenticatorContext, Resources, ResourcesContext } from '../../app-config';
import { AppController, AppControllerContext } from '../app-contexts';
import AppBar from '../appbar';
import { render } from '../tests/test-utils';
import { makeMockAppController } from './mock-app-controller';

describe('AppBar', () => {
  let appController: AppController;
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <AppControllerContext.Provider value={appController}>
        {props.children}
      </AppControllerContext.Provider>
    );
  };

  beforeEach(() => {
    appController = makeMockAppController();
  });

  test('renders with navigation bar', () => {
    const root = render(
      <Base>
        <AppBar />
      </Base>,
    );
    expect(root.getAllByRole('tablist').length > 0).toBeTruthy();
  });

  test('user button is shown when there is an authenticated user', () => {
    const profile: UserProfile = {
      user: { username: 'test', is_admin: false, roles: [] },
      permissions: [],
    };
    const root = render(
      <Base>
        <UserProfileContext.Provider value={profile}>
          <AppBar />
        </UserProfileContext.Provider>
      </Base>,
    );
    expect(root.getByLabelText('user-btn')).toBeTruthy();
  });

  test('logout is triggered when logout button is clicked', async () => {
    const authenticator = new StubAuthenticator('test');
    const spy = jest.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);
    const profile: UserProfile = {
      user: { username: 'test', is_admin: false, roles: [] },
      permissions: [],
    };
    const root = render(
      <AuthenticatorContext.Provider value={authenticator}>
        <Base>
          <UserProfileContext.Provider value={profile}>
            <AppBar />
          </UserProfileContext.Provider>
        </Base>
      </AuthenticatorContext.Provider>,
    );
    userEvent.click(root.getByLabelText('user-btn'));
    await expect(waitFor(() => root.getByText('Logout'))).resolves.not.toThrow();
    userEvent.click(root.getByText('Logout'));
    await expect(waitFor(() => expect(spy).toHaveBeenCalledTimes(1))).resolves.not.toThrow();
  });

  test('uses headerLogo from logo resources manager', async () => {
    const resources: Resources = {
      fleets: {},
      logos: {
        header: '/test-logo.png',
      },
    };

    const root = render(
      <ResourcesContext.Provider value={resources}>
        <Base>
          <AppBar />
        </Base>
      </ResourcesContext.Provider>,
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
