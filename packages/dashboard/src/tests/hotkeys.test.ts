import { ReducerMainMenuProps, useMainMenuReducer } from '../components/reducers/main-menu-reducer';
import { buildHotKeys, keyMap } from '../hotkeys';
import { HotKeysEnabledProps } from 'react-hotkeys';
import { act, HookResult, renderHook } from '@testing-library/react-hooks';
import { mainMenuInitialValues, OmniPanelViewIndex } from '../components/dashboard';

test('Build hotkeys on the correct format', () => {
  const { result } = renderHook(() => useMainMenuReducer(mainMenuInitialValues));
  const hotkeys = buildHotKeys({ dispatch: result.current.dispatch });
  if (!hotkeys.keyMap) throw new Error('An error has occurred building the hotkeys formats');
  expect(Object.keys(hotkeys.keyMap)).toEqual(Object.keys(keyMap));
});

describe('Update states correctly', () => {
  let hotkeys: HotKeysEnabledProps;
  let result: HookResult<ReducerMainMenuProps>;

  beforeEach(() => {
    const hookResult = renderHook(() => useMainMenuReducer(mainMenuInitialValues));
    result = hookResult.result;
    hotkeys = buildHotKeys({ dispatch: result.current.dispatch });
  });

  test('Set commands as current view', () => {
    act(() => {
      hotkeys.handlers?.OPEN_COMMANDS();
    });
    expect(result.current.state.currentView).toBe(OmniPanelViewIndex.Commands);
  });

  test('Set dispensers as current view', () => {
    act(() => {
      hotkeys.handlers?.OPEN_DISPENSERS();
    });
    expect(result.current.state.currentView).toBe(OmniPanelViewIndex.Dispensers);
  });

  test('Set doors as current view', () => {
    act(() => {
      hotkeys.handlers?.OPEN_DOORS();
    });
    expect(result.current.state.currentView).toBe(OmniPanelViewIndex.Doors);
  });

  test('Set lifts current view correctly', () => {
    act(() => {
      hotkeys.handlers?.OPEN_LIFTS();
    });
    expect(result.current.state.currentView).toBe(OmniPanelViewIndex.Lifts);
  });

  test('Set robots current view correctly', () => {
    act(() => {
      hotkeys.handlers?.OPEN_ROBOTS();
    });
    expect(result.current.state.currentView).toBe(OmniPanelViewIndex.Robots);
  });

  test('Toggle Help Panel correctly', () => {
    act(() => {
      hotkeys.handlers?.OPEN_HELP_PANEL();
    });

    expect(result.current.state.showHelp).toBe(true);

    act(() => {
      hotkeys.handlers?.OPEN_HELP_PANEL();
    });
    expect(result.current.state.showHelp).toBe(false);
  });

  test('Toggle Omni Panel correctly', () => {
    act(() => {
      hotkeys.handlers?.OPEN_OMNIPANEL();
    });

    expect(result.current.state.showOmniPanel).toBe(false);

    act(() => {
      hotkeys.handlers?.OPEN_OMNIPANEL();
    });
    expect(result.current.state.showOmniPanel).toBe(true);
  });

  test('Toggle Setting correctly', () => {
    act(() => {
      hotkeys.handlers?.OPEN_SETTINGS();
    });

    expect(result.current.state.showSettings).toBe(true);

    act(() => {
      hotkeys.handlers?.OPEN_SETTINGS();
    });
    expect(result.current.state.showSettings).toBe(false);
  });
});
