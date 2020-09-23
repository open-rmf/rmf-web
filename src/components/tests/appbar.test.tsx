import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import FakeAuthenticator from '../../mock/fake-authenticator';
import AppBar from '../appbar';
import { AuthenticatorContext, UserContext } from '../auth/contexts';

const mount = createMount();

describe('AppBar', () => {
  let toggleShowOmniPanel: jest.Mock;
  let showSettings: jest.Mock;
  let showTour: jest.Mock;

  beforeEach(() => {
    toggleShowOmniPanel = jest.fn();
    showSettings = jest.fn();
    showTour = jest.fn();
  });

  test('renders correctly', () => {
    const root = mount(
      <AppBar
        toggleShowOmniPanel={toggleShowOmniPanel}
        showSettings={showSettings}
        showTour={showTour}
      />,
    );
    expect(root).toMatchSnapshot();
  });

  test('toggles show omnipanel when dashboard button is clicked', () => {
    const root = mount(
      <AppBar
        toggleShowOmniPanel={toggleShowOmniPanel}
        showSettings={showSettings}
        showTour={showTour}
      />,
    );
    root.find('button#toggle-omnipanel-btn').simulate('click');
    expect(toggleShowOmniPanel).toHaveBeenCalledTimes(1);
  });

  test('show settings when settings button is clicked', () => {
    const root = mount(
      <AppBar
        toggleShowOmniPanel={toggleShowOmniPanel}
        showSettings={showSettings}
        showTour={showTour}
      />,
    );
    root.find('button#show-settings-btn').simulate('click');
    expect(showSettings).toHaveBeenCalledTimes(1);
  });

  test('show tour when tour button is clicked', () => {
    const root = mount(
      <AppBar
        toggleShowOmniPanel={toggleShowOmniPanel}
        showSettings={showSettings}
        showTour={showTour}
      />,
    );
    root.find('button#show-tour-btn').simulate('click');
    expect(showTour).toHaveBeenCalledTimes(1);
  });

  test('user button is shown when there is an authenticated user', () => {
    const root = mount(
      <UserContext.Provider value={{ username: 'test' }}>
        <AppBar
          toggleShowOmniPanel={toggleShowOmniPanel}
          showSettings={showSettings}
          showTour={showTour}
        />
      </UserContext.Provider>,
    );
    expect(root.find('#user-btn').length > 0).toBeTruthy();
  });

  test('logout is triggered when logout button is clicked', () => {
    const authenticator = new FakeAuthenticator();
    const spy = jest.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);

    const root = mount(
      <AuthenticatorContext.Provider value={authenticator}>
        <UserContext.Provider value={{ username: 'test' }}>
          <AppBar
            toggleShowOmniPanel={toggleShowOmniPanel}
            showSettings={showSettings}
            showTour={showTour}
          />
        </UserContext.Provider>
      </AuthenticatorContext.Provider>,
    );
    root.find('button#user-btn').simulate('click');
    root.find('li#logout-btn').simulate('click');
    expect(spy).toHaveBeenCalledTimes(1);
  });
});
