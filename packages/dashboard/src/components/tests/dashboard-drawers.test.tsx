import { render, RenderResult } from '@testing-library/react';
import { act, HookResult, renderHook } from '@testing-library/react-hooks';
import React from 'react';
import { mainMenuInitialValues } from '../dashboard';
import { DashboardDrawers } from '../dashboard-drawers';
import { ReducerMainMenuProps, useMainMenuReducer } from '../reducers/main-menu-reducer';

describe('dashboard-drawers', () => {
  let result: HookResult<ReducerMainMenuProps>;
  let root: RenderResult;

  beforeEach(() => {
    const hookResult = renderHook(() => useMainMenuReducer(mainMenuInitialValues));
    result = hookResult.result;
  });

  test('setting`s drawer is shown', () => {
    act(() => {
      result.current.dispatch.setShowSettings(true);
    });
    const state = result.current.state;
    root = render(
      <DashboardDrawers
        settings={state.settings}
        showSettings={state.showSettings}
        showHelp={state.showHelp}
        showHotkeysDialog={state.showHotkeysDialog}
        reducerMainMenuDispatch={result.current.dispatch}
      />,
    );
    expect(root.getByText('Settings')).toBeTruthy();
  });

  test('help`s drawer is shown', () => {
    act(() => {
      result.current.dispatch.setShowHelp(true);
    });
    const state = result.current.state;
    root = render(
      <DashboardDrawers
        settings={state.settings}
        showSettings={state.showSettings}
        showHelp={state.showHelp}
        showHotkeysDialog={state.showHotkeysDialog}
        reducerMainMenuDispatch={result.current.dispatch}
      />,
    );
    expect(root.getByText('Help')).toBeTruthy();
  });

  test('hotkeys` drawer is shown', () => {
    act(() => {
      result.current.dispatch.setShowHotkeysDialog(true);
    });
    const state = result.current.state;
    root = render(
      <DashboardDrawers
        settings={state.settings}
        showSettings={state.showSettings}
        showHelp={state.showHelp}
        showHotkeysDialog={state.showHotkeysDialog}
        reducerMainMenuDispatch={result.current.dispatch}
      />,
    );
    expect(root.getByText('Hotkeys')).toBeTruthy();
  });
});
