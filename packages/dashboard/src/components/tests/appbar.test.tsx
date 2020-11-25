import { createMount, createShallow } from '@material-ui/core/test-utils';
import { shallow } from 'enzyme';
import React from 'react';
import FakeAuthenticator from '../../mock/fake-authenticator';
import AppBar from '../appbar';
import { AuthenticatorContext, UserContext } from '../auth/contexts';

const mount = createMount();

const buildWrapper = (dispatchMenu: () => void) => {
  const root = mount(<AppBar dispatchMenu={dispatchMenu} alarmState={null} />);
  return root;
};

describe('AppBar', () => {
  let toggleShowOmniPanel: jest.Mock;
  let showSettings: jest.Mock;
  let showHelp: jest.Mock;
  let dispatchMenu: jest.Mock;

  beforeEach(() => {
    toggleShowOmniPanel = jest.fn();
    showSettings = jest.fn();
    showHelp = jest.fn();
    dispatchMenu = jest.fn();
  });

  test('renders correctly', () => {
    const root = shallow(<AppBar dispatchMenu={dispatchMenu} alarmState={null} />);
    expect(root).toMatchSnapshot();
  });

  test('renders tooltips when it is enabled', () => {
    const root = buildWrapper(dispatchMenu);
    expect(root.find('#omnipanel-tooltip').exists()).toBeTruthy();
    expect(root.find('#setting-tooltip').exists()).toBeTruthy();
    expect(root.find('#help-tooltip').exists()).toBeTruthy();
  });

  test('toggles show omnipanel when dashboard button is clicked', () => {
    const root = buildWrapper(dispatchMenu);
    root.find('button#toggle-omnipanel-btn').simulate('click');
    expect(dispatchMenu).toHaveBeenCalledTimes(1);
  });

  test('show settings when settings button is clicked', () => {
    const root = buildWrapper(dispatchMenu);
    root.find('button#show-settings-btn').simulate('click');
    expect(dispatchMenu).toHaveBeenCalledTimes(1);
  });

  test('show help when help button is clicked', () => {
    const root = buildWrapper(dispatchMenu);
    root.find('button#show-help-btn').simulate('click');
    expect(dispatchMenu).toHaveBeenCalledTimes(1);
  });

  test('user button is shown when there is an authenticated user', () => {
    const dispatchMenu = jest.fn();
    const root = mount(
      <UserContext.Provider value={{ username: 'test' }}>
        <AppBar dispatchMenu={dispatchMenu} alarmState={null} />
      </UserContext.Provider>,
    );
    expect(root.find('#user-btn').length > 0).toBeTruthy();
  });

  test('logout is triggered when logout button is clicked', () => {
    const authenticator = new FakeAuthenticator();
    const spy = jest.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);
    const dispatchMenu = jest.fn();
    const root = mount(
      <AuthenticatorContext.Provider value={authenticator}>
        <UserContext.Provider value={{ username: 'test' }}>
          <AppBar dispatchMenu={dispatchMenu} alarmState={null} />
        </UserContext.Provider>
      </AuthenticatorContext.Provider>,
    );
    root.find('button#user-btn').simulate('click');
    root.find('li#logout-btn').simulate('click');
    expect(spy).toHaveBeenCalledTimes(1);
  });
});
