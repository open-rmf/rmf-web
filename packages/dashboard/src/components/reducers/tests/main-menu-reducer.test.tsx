import { act, renderHook } from '@testing-library/react-hooks';
import React from 'react';
import { loadSettings } from '../../../settings';
import {
  MainMenuAction,
  MainMenuActionType,
  mainMenuReducer,
  MainMenuState,
  useMainMenu,
} from '../main-menu-reducer';

const mainMenuInitialValues: MainMenuState = {
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

const getReducer = () => {
  const { result } = renderHook(() => useMainMenu(mainMenuInitialValues));
  return result.current;
};

describe('Main Menu reducer update states correctly', () => {
  let state: MainMenuState, dispatch: React.Dispatch<MainMenuAction>;
  // beforeEach(() => {
  //   const { result } = renderHook(() => React.useReducer(mainMenuReducer, mainMenuInitialValues));

  // });

  // test('Update current view', () => {
  //   [state, dispatch] = getReducer();
  //   act(() => {
  //     dispatch({ type: MainMenuActionType.CurrentView, payload: 2 });
  //   });
  //   expect(state.currentView).toBe(2);
  // });

  test('Update showHelp state', async () => {
    // const { result, waitForNextUpdate } = renderHook(() =>
    //   React.useReducer(mainMenuReducer, mainMenuInitialValues),
    // );
    const { result } = renderHook(() => useMainMenu(mainMenuInitialValues));
    // const reducer = getReducer();
    console.log(result.current.showHelp);
    act(() => {
      result.current.setShowHelp(true);
      // reducer.setShowHelp(true);
    });
    console.log(result.current.showHelp);

    expect(result.current.showHelp).toBe(true);
  });
});
