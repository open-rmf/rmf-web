import { render, RenderResult } from '@testing-library/react';
import { act, HookResult, renderHook } from '@testing-library/react-hooks';
import React from 'react';
import { DashboardDrawers } from '../dashboard-drawers';
import { ReducerMainMenuProps, useMainMenuReducer } from '../reducers/main-menu-reducer';
import { mainMenuInitialValues } from '../reducers/main-menu-reducer-initial-values';

describe('Dashboard-drawers', () => {
  let result: HookResult<ReducerMainMenuProps>;
  let root: RenderResult;

  beforeEach(() => {
    const hookResult = renderHook(() => useMainMenuReducer(mainMenuInitialValues));
    result = hookResult.result;
  });

  test('Setting drawer is shown', () => {
    act(() => {
      result.current.setShowSettings(true);
    });
    root = render(<DashboardDrawers reducerMainMenu={result.current} />);
    expect(root.getByText('Settings')).toBeTruthy();
  });

  test('Help drawer is shown', () => {
    act(() => {
      result.current.setShowHelp(true);
    });
    root = render(<DashboardDrawers reducerMainMenu={result.current} />);
    expect(root.getByText('Help')).toBeTruthy();
  });

  test('Hotkeys drawer is shown', () => {
    act(() => {
      result.current.setShowHotkeysDialog(true);
    });
    root = render(<DashboardDrawers reducerMainMenu={result.current} />);
    expect(root.getByText('Hotkeys')).toBeTruthy();
  });
});
