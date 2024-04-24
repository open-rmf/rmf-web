import { waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { StubAuthenticator, UserProfile, UserProfileContext } from 'rmf-auth';
import { AppConfig } from '../../app-config';
import ResourceManager from '../../managers/resource-manager';
import { LogoResourceManager } from '../../managers/resource-manager-logos';
import { RobotResourceManager } from '../../managers/resource-manager-robots';
import {
  AppConfigContext,
  AppController,
  AppControllerContext,
  ResourcesContext,
} from '../app-contexts';
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
    const appConfig: AppConfig = {
      authenticator,
      appResourcesFactory: jest.fn(),
      rmfServerUrl: '',
      trajServerUrl: '',
    };
    const spy = jest.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);
    const profile: UserProfile = {
      user: { username: 'test', is_admin: false, roles: [] },
      permissions: [],
    };
    const root = render(
      <AppConfigContext.Provider value={appConfig}>
        <Base>
          <UserProfileContext.Provider value={profile}>
            <AppBar />
          </UserProfileContext.Provider>
        </Base>
      </AppConfigContext.Provider>,
    );
    await userEvent.click(root.getByLabelText('user-btn'));
    await userEvent.click(root.getByText('Logout'));
    expect(spy).toHaveBeenCalledTimes(1);
  });

  test('uses headerLogo from logo resources manager', async () => {
    const robotResourcesMgr = new RobotResourceManager({});
    const logoResourcesMgr = new LogoResourceManager({});
    logoResourcesMgr.getHeaderLogoPath = () => Promise.resolve('/test-logo.png');
    const resourceMgr: ResourceManager = {
      robots: robotResourcesMgr,
      logos: logoResourcesMgr,
      helpLink: '',
      reportIssue: '',
    };

    const root = render(
      <ResourcesContext.Provider value={resourceMgr}>
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
