import { act, HookResult, renderHook } from '@testing-library/react-hooks';
import { mainMenuInitialValues } from '../main-menu-reducer-initial-values';
import { ReducerMainMenuProps, useMainMenuReducer } from '../main-menu-reducer';

describe('Main Menu reducer update states correctly', () => {
  let result: HookResult<ReducerMainMenuProps>;
  beforeEach(() => {
    const hookResult = renderHook(() => useMainMenuReducer(mainMenuInitialValues));
    result = hookResult.result;
  });

  test('Update current view correctly', async () => {
    act(() => {
      result.current.setCurrentView(2);
    });
    expect(result.current.currentView).toBe(2);
  });

  test('Update showHelp state correctly', async () => {
    act(() => {
      result.current.setShowHelp(true);
    });
    expect(result.current.showHelp).toBe(true);
  });

  test('Update showHotkeysDialog correctly', async () => {
    act(() => {
      result.current.setShowHotkeysDialog(true);
    });
    expect(result.current.showHotkeysDialog).toBe(true);
  });

  test('Update showOmniPanel correctly', async () => {
    act(() => {
      result.current.setShowOmniPanel(false);
    });
    expect(result.current.showOmniPanel).toBe(false);
  });

  test('Update showSettings correctly', async () => {
    act(() => {
      result.current.setShowSettings(true);
    });
    expect(result.current.showSettings).toBe(true);
  });

  test('Update tourState correctly', async () => {
    act(() => {
      result.current.setTourState(true);
    });
    expect(result.current.tourState).toBe(true);
  });

  test('Toggle showHelp correctly', async () => {
    act(() => {
      result.current.toggleHelp();
    });
    expect(result.current.showHelp).toBe(true);
  });

  test('Toggle ToggleHotkeys correctly', async () => {
    act(() => {
      result.current.toggleHotkeys();
    });
    expect(result.current.showHotkeysDialog).toBe(true);
  });

  test('Toggle ShowOmniPanel correctly', async () => {
    act(() => {
      result.current.toggleOmnipanel();
    });
    expect(result.current.showOmniPanel).toBe(false);
  });

  test('Toggle ShowSettings correctly', async () => {
    act(() => {
      result.current.toggleSettings();
    });
    expect(result.current.showSettings).toBe(true);
  });
});
