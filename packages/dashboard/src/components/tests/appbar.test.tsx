import { createMount } from '@material-ui/core/test-utils';
import { act, HookResult, renderHook } from '@testing-library/react-hooks';
import { shallow } from 'enzyme';
import React from 'react';
import { render, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';

import FakeAuthenticator from '../../mock/fake-authenticator';
import AppBar from '../appbar';
import { AuthenticatorContext, UserContext } from '../auth/contexts';
import { ReducerMainMenuProps, useMainMenuReducer } from '../reducers/main-menu-reducer';
import { mainMenuInitialValues } from '../reducers/main-menu-reducer-initial-values';

describe('AppBar', () => {
  /**
   * Since we do not have visual elements to test in this component (change of button color, etc).
   * We test the correct change of state through events. Because it is a unit test we cannot test
   * the connection with other components here (e.g. omnipanel).
   */
  let result: HookResult<ReducerMainMenuProps>;
  let root: RenderResult;

  beforeEach(() => {
    const hookResult = renderHook(() => useMainMenuReducer(mainMenuInitialValues));
    result = hookResult.result;
    root = render(<AppBar reducerMainMenu={result.current} alarmState={null} />);
  });

  // Should we keep the snapshot test?
  test('renders correctly', () => {
    const shallowRoot = shallow(<AppBar reducerMainMenu={result.current} alarmState={null} />);
    expect(shallowRoot).toMatchSnapshot();
  });

  test('toggles hides omnipanel when dashboard button is clicked', () => {
    act(() => {
      const elements = root.getAllByTestId('omnipanel-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(result.current.showOmniPanel).toBe(false);
  });

  test('show settings when settings button is clicked', () => {
    act(() => {
      const elements = root.getAllByTestId('setting-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(result.current.showSettings).toBe(true);
  });

  test('show help when help button is clicked', () => {
    act(() => {
      const elements = root.getAllByTestId('help-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(result.current.showHelp).toBe(true);
  });

  test('renders tooltips when it is enabled', async () => {
    userEvent.hover(root.getByTestId('help-tooltip-tooltip'));
    expect(await root.findByText('Help tools and resources')).toBeTruthy();

    userEvent.hover(root.getByTestId('omnipanel-tooltip-tooltip'));
    expect(await root.findByText('View all available panel options')).toBeTruthy();

    userEvent.hover(root.getByTestId('setting-tooltip-tooltip'));
    expect(await root.findByText('Define dashboard trajectory settings')).toBeTruthy();
  });

  test('user button is shown when there is an authenticated user', () => {
    const root = render(
      <UserContext.Provider value={{ username: 'test' }}>
        <AppBar reducerMainMenu={result.current} alarmState={null} />
      </UserContext.Provider>,
    );
    expect(root.getByLabelText('user-btn')).toBeTruthy();
  });

  test('logout is triggered when logout button is clicked', () => {
    const authenticator = new FakeAuthenticator();
    const spy = jest.spyOn(authenticator, 'logout').mockImplementation(() => undefined as any);
    const root = render(
      <AuthenticatorContext.Provider value={authenticator}>
        <UserContext.Provider value={{ username: 'test' }}>
          <AppBar reducerMainMenu={result.current} alarmState={null} />
        </UserContext.Provider>
      </AuthenticatorContext.Provider>,
    );
    userEvent.click(root.getByLabelText('user-btn'));
    userEvent.click(root.getByText('Logout'));
    // root.find('button#user-btn').simulate('click');
    // root.find('li#logout-btn').simulate('click');
    expect(spy).toHaveBeenCalledTimes(1);
  });
});
