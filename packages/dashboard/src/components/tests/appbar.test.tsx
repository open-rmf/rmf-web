import { ThemeProvider } from '@material-ui/core';
import { render, waitFor } from '@testing-library/react';
import { act } from '@testing-library/react-hooks';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { MemoryRouter } from 'react-router';
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
import { makeMockAppController } from './mock-app-controller';
import { mountAsUser, mockTheme } from './test-utils';

describe('AppBar', () => {
  let appController: AppController;
  const Base = (props: React.PropsWithChildren<{}>) => {
    return (
      <MemoryRouter>
        <ThemeProvider theme={mockTheme}>
          <AppControllerContext.Provider value={appController}>
            {props.children}
          </AppControllerContext.Provider>
        </ThemeProvider>
      </MemoryRouter>
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

  test('shows help when help button is clicked', () => {
    const root = render(
      <Base>
        <AppBar />
      </Base>,
    );
    act(() => {
      const elements = root.getAllByTestId('help-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(appController.showHelp).toBeCalledTimes(1);
  });

  test('renders tooltips when it is enabled', async () => {
    const root = render(
      <Base>
        <AppBar />
      </Base>,
    );
    userEvent.hover(root.getByTestId('help-tooltip-tooltip'));
    expect(await root.findByText('Help tools and resources')).toBeTruthy();
  });

  test('user button is shown when there is an authenticated user', () => {
    const profile: UserProfile = {
      user: { username: 'test', is_admin: false, roles: [] },
      permissions: [],
    };
    const root = mountAsUser(
      profile,
      <Base>
        <AppBar />
      </Base>,
    );
    expect(root.getByLabelText('user-btn')).toBeTruthy();
  });

  test('logout is triggered when logout button is clicked', () => {
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
    userEvent.click(root.getByLabelText('user-btn'));
    userEvent.click(root.getByText('Logout'));
    expect(spy).toHaveBeenCalledTimes(1);
  });

  test('uses headerLogo from logo resources manager', async () => {
    const robotResourcesMgr = new RobotResourceManager({});
    const logoResourcesMgr = new LogoResourceManager({});
    logoResourcesMgr.getHeaderLogoPath = () => Promise.resolve('/test-logo.png');
    const resourceMgr: ResourceManager = { robots: robotResourcesMgr, logos: logoResourcesMgr };

    const root = render(
      <ResourcesContext.Provider value={resourceMgr}>
        <AppBar />
      </ResourcesContext.Provider>,
      { wrapper: Base },
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
