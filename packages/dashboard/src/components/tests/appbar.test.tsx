import { createMount } from '@material-ui/core/test-utils';
import { act, HookResult, renderHook } from '@testing-library/react-hooks';
import { shallow } from 'enzyme';
import React from 'react';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';

import FakeAuthenticator from '../../mock/fake-authenticator';
import AppBar from '../appbar';
import { AuthenticatorContext, UserContext } from '../auth/contexts';
import { ReducerMainMenuProps, useMainMenu } from '../reducers/main-menu-reducer';
import { mainMenuInitialValues } from '../reducers/main-menu-reducer-initial-values';

const mount = createMount();

const buildWrapper = (reducerMainMenu: ReducerMainMenuProps) => {
  const root = mount(<AppBar reducerMainMenu={reducerMainMenu} alarmState={null} />);
  return root;
};

describe('AppBar', () => {
  /**
   * Since we do not have visual elements to test in this component (change of button color, etc).
   * We test the correct change of state through events. Because it is a unit test we cannot test
   * the connection with other components here (e.g. omnipanel).
   */
  let result: HookResult<ReducerMainMenuProps>;

  beforeEach(() => {
    const hookResult = renderHook(() => useMainMenu(mainMenuInitialValues));
    result = hookResult.result;
  });

  test('toggles hides omnipanel when dashboard button is clicked', () => {
    const root = render(<AppBar reducerMainMenu={result.current} alarmState={null} />);
    act(() => {
      const elements = root.getAllByTestId('omnipanel-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(result.current.showOmniPanel).toBe(false);
  });

  test('show settings when settings button is clicked', () => {
    const root = render(<AppBar reducerMainMenu={result.current} alarmState={null} />);
    act(() => {
      const elements = root.getAllByTestId('setting-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(result.current.showSettings).toBe(true);
  });

  test('show help when help button is clicked', () => {
    const root = render(<AppBar reducerMainMenu={result.current} alarmState={null} />);
    act(() => {
      const elements = root.getAllByTestId('help-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(result.current.showHelp).toBe(true);
  });

  test('user button is shown when there is an authenticated user', () => {
    const root = mount(
      <UserContext.Provider value={{ username: 'test' }}>
        <AppBar reducerMainMenu={result.current} alarmState={null} />
      </UserContext.Provider>,
    );
    expect(root.find('#user-btn').length > 0).toBeTruthy();
  });

  test('renders correctly', () => {
    const root = shallow(<AppBar reducerMainMenu={result.current} alarmState={null} />);
    expect(root).toMatchSnapshot();
  });

  test('renders tooltips when it is enabled', () => {
    const root = buildWrapper(result.current);
    expect(root.find('#omnipanel-tooltip').exists()).toBeTruthy();
    expect(root.find('#setting-tooltip').exists()).toBeTruthy();
    expect(root.find('#help-tooltip').exists()).toBeTruthy();
  });

  test('logout is triggered when logout button is clicked', () => {
    const authenticator = new FakeAuthenticator();
    const spy = jest.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);
    const root = mount(
      <AuthenticatorContext.Provider value={authenticator}>
        <UserContext.Provider value={{ username: 'test' }}>
          <AppBar reducerMainMenu={result.current} alarmState={null} />
        </UserContext.Provider>
      </AuthenticatorContext.Provider>,
    );
    root.find('button#user-btn').simulate('click');
    root.find('li#logout-btn').simulate('click');
    expect(spy).toHaveBeenCalledTimes(1);
  });
});
