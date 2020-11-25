import { createMount, createShallow } from '@material-ui/core/test-utils';
import { renderHook } from '@testing-library/react-hooks';
import { shallow } from 'enzyme';
import React from 'react';
import FakeAuthenticator from '../../mock/fake-authenticator';
import { loadSettings } from '../../settings';
import AppBar from '../appbar';
import { AuthenticatorContext, UserContext } from '../auth/contexts';
import { ReducerMainMenuProps, useMainMenu } from '../reducers/main-menu-reducer';

const mount = createMount();

const buildWrapper = (reducerMainMenu: ReducerMainMenuProps) => {
  const root = mount(<AppBar reducerMainMenu={reducerMainMenu} alarmState={null} />);
  return root;
};

describe('AppBar', () => {
  let reducerMainMenu: ReducerMainMenuProps;

  beforeEach(() => {
    const mainMenuInitialValues = {
      currentView: 1,
      loading: {
        caption: 'Connecting to api server...',
      },
      settings: loadSettings(),
      showHelp: false,
      showHotkeysDialog: false,
      showOmniPanel: true,
      showSettings: false,
      tourState: false,
    };
    const { result } = renderHook(() => useMainMenu(mainMenuInitialValues));
    reducerMainMenu = result.current;
  });

  test('renders correctly', () => {
    const root = shallow(<AppBar reducerMainMenu={reducerMainMenu} alarmState={null} />);
    expect(root).toMatchSnapshot();
  });

  test('renders tooltips when it is enabled', () => {
    const root = buildWrapper(reducerMainMenu);
    expect(root.find('#omnipanel-tooltip').exists()).toBeTruthy();
    expect(root.find('#setting-tooltip').exists()).toBeTruthy();
    expect(root.find('#help-tooltip').exists()).toBeTruthy();
  });

  // test('toggles show omnipanel when dashboard button is clicked', () => {
  //   const root = buildWrapper(reducerMainMenu);
  //   root.find('button#toggle-omnipanel-btn').simulate('click');
  //   expect(reducerMainMenu).toHaveBeenCalledTimes(1);
  // });

  // test('show settings when settings button is clicked', () => {
  //   const root = buildWrapper(reducerMainMenu);
  //   root.find('button#show-settings-btn').simulate('click');
  //   expect(reducerMainMenu).toHaveBeenCalledTimes(1);
  // });

  // test('show help when help button is clicked', () => {
  //   const root = buildWrapper(reducerMainMenu);
  //   root.find('button#show-help-btn').simulate('click');
  //   expect(reducerMainMenu).toHaveBeenCalledTimes(1);
  // });

  test('user button is shown when there is an authenticated user', () => {
    const root = mount(
      <UserContext.Provider value={{ username: 'test' }}>
        <AppBar reducerMainMenu={reducerMainMenu} alarmState={null} />
      </UserContext.Provider>,
    );
    expect(root.find('#user-btn').length > 0).toBeTruthy();
  });

  // test('logout is triggered when logout button is clicked', () => {
  //   const authenticator = new FakeAuthenticator();
  //   const spy = jest.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);
  //   const reducerMainMenu = jest.fn();
  //   const root = mount(
  //     <AuthenticatorContext.Provider value={authenticator}>
  //       <UserContext.Provider value={{ username: 'test' }}>
  //         <AppBar reducerMainMenu={reducerMainMenu} alarmState={null} />
  //       </UserContext.Provider>
  //     </AuthenticatorContext.Provider>,
  //   );
  //   root.find('button#user-btn').simulate('click');
  //   root.find('li#logout-btn').simulate('click');
  //   expect(spy).toHaveBeenCalledTimes(1);
  // });
});
