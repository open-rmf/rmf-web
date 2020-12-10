import { act, HookResult, renderHook } from '@testing-library/react-hooks';
import React from 'react';
import { render, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import FakeAuthenticator from '../../mock/fake-authenticator';
import AppBar from '../appbar';
import { AuthenticatorContext, UserContext } from '../auth/contexts';
import { ReducerMainMenuProps, useMainMenuReducer } from '../reducers/main-menu-reducer';
import { mainMenuInitialValues } from '../dashboard';

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
    root = render(<AppBar reducerMainMenuDispatch={result.current.dispatch} alarmState={null} />);
  });

  test('hides omnipanel when dashboard button is clicked', () => {
    act(() => {
      const elements = root.getAllByTestId('omnipanel-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(result.current.state.showOmniPanel).toBe(false);
  });

  test('show settings when settings button is clicked', () => {
    act(() => {
      const elements = root.getAllByTestId('setting-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(result.current.state.showSettings).toBe(true);
  });

  test('shows help when help button is clicked', () => {
    act(() => {
      const elements = root.getAllByTestId('help-tooltip-tooltip');
      for (let element in elements) {
        userEvent.click(elements[element]);
      }
    });
    expect(result.current.state.showHelp).toBe(true);
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
        <AppBar reducerMainMenuDispatch={result.current.dispatch} alarmState={null} />
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
          <AppBar reducerMainMenuDispatch={result.current.dispatch} alarmState={null} />
        </UserContext.Provider>
      </AuthenticatorContext.Provider>,
    );
    userEvent.click(root.getByLabelText('user-btn'));
    userEvent.click(root.getByText('Logout'));
    expect(spy).toHaveBeenCalledTimes(1);
  });
});
