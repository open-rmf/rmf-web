import { ThemeProvider } from '@material-ui/core';
import { render } from '@testing-library/react';
import { act } from '@testing-library/react-hooks';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { MemoryRouter } from 'react-router';
import { AppConfig } from '../../app-config';
import { AppConfigContext, AppController, AppControllerContext } from '../app-contexts';
import AppBar from '../appbar';
import { User, UserContext } from '../auth/contexts';
import FakeAuthenticator from '../auth/__mocks__/fake-authenticator';
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

  test('show settings when settings button is clicked', () => {
    const root = render(
      <Base>
        <AppBar />
      </Base>,
    );
    act(() => {
      const elements = root.getAllByTestId('setting-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(appController.showSettings).toBeCalledTimes(1);
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

    userEvent.hover(root.getByTestId('setting-tooltip-tooltip'));
    expect(await root.findByText('Define dashboard trajectory settings')).toBeTruthy();
  });

  test('user button is shown when there is an authenticated user', () => {
    const user: User = {
      profile: { username: 'test', is_admin: false, roles: [] },
      permissions: [],
    };
    const root = mountAsUser(
      user,
      <Base>
        <AppBar />
      </Base>,
    );
    expect(root.getByLabelText('user-btn')).toBeTruthy();
  });

  test('logout is triggered when logout button is clicked', () => {
    const authenticator = new FakeAuthenticator('test');
    const appConfig: AppConfig = {
      authenticator,
      appResourcesFactory: jest.fn(),
      rmfServerUrl: '',
      trajServerUrl: '',
    };
    const spy = jest.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);
    const user: User = {
      profile: { username: 'test', is_admin: false, roles: [] },
      permissions: [],
    };
    const root = render(
      <AppConfigContext.Provider value={appConfig}>
        <Base>
          <UserContext.Provider value={user}>
            <AppBar />
          </UserContext.Provider>
        </Base>
      </AppConfigContext.Provider>,
    );
    userEvent.click(root.getByLabelText('user-btn'));
    userEvent.click(root.getByText('Logout'));
    expect(spy).toHaveBeenCalledTimes(1);
  });
});
