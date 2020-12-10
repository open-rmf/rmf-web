import { act, HookResult, renderHook } from '@testing-library/react-hooks';
import { mainMenuInitialValues } from '../../dashboard';
import { ReducerMainMenuProps, useMainMenuReducer } from '../main-menu-reducer';

describe('Main Menu reducer update states correctly', () => {
  let result: HookResult<ReducerMainMenuProps>;
  beforeEach(() => {
    const hookResult = renderHook(() => useMainMenuReducer(mainMenuInitialValues));
    result = hookResult.result;
  });

  test('Update current view correctly', async () => {
    act(() => {
      result.current.dispatch.setCurrentView(2);
    });
    expect(result.current.state.currentView).toBe(2);
  });

  test('Update showHelp state correctly', async () => {
    act(() => {
      result.current.dispatch.setShowHelp(true);
    });
    expect(result.current.state.showHelp).toBe(true);
  });

  test('Update showHotkeysDialog correctly', async () => {
    act(() => {
      result.current.dispatch.setShowHotkeysDialog(true);
    });
    expect(result.current.state.showHotkeysDialog).toBe(true);
  });

  test('Update showOmniPanel correctly', async () => {
    act(() => {
      result.current.dispatch.setShowOmniPanel(false);
    });
    expect(result.current.state.showOmniPanel).toBe(false);
  });

  test('Update showSettings correctly', async () => {
    act(() => {
      result.current.dispatch.setShowSettings(true);
    });
    expect(result.current.state.showSettings).toBe(true);
  });

  test('Update tourState correctly', async () => {
    act(() => {
      result.current.dispatch.setTourState(true);
    });
    expect(result.current.state.tourState).toBe(true);
  });

  test('Toggle showHelp correctly', async () => {
    act(() => {
      result.current.dispatch.toggleHelp();
    });
    expect(result.current.state.showHelp).toBe(true);
  });

  test('Toggle ToggleHotkeys correctly', async () => {
    act(() => {
      result.current.dispatch.toggleHotkeys();
    });
    expect(result.current.state.showHotkeysDialog).toBe(true);
  });

  test('Toggle ShowOmniPanel correctly', async () => {
    act(() => {
      result.current.dispatch.toggleOmnipanel();
    });
    expect(result.current.state.showOmniPanel).toBe(false);
  });

  test('Toggle ShowSettings correctly', async () => {
    act(() => {
      result.current.dispatch.toggleSettings();
    });
    expect(result.current.state.showSettings).toBe(true);
  });

  test('Toggle ShowSettings correctly', async () => {
    act(() => {
      result.current.dispatch.toggleSettings();
    });
    expect(result.current.state.showSettings).toBe(true);
  });

  test('`pushView` updates stackNavigator and currentView correctly', async () => {
    act(() => {
      result.current.dispatch.pushView(3);
    });
    expect(result.current.state.currentView).toBe(3);
    expect(result.current.state.stackNavigator.top()).toBe(3);
  });

  test('`popView` pops elements from stackNavigator', async () => {
    act(() => {
      result.current.dispatch.pushView(3);
      result.current.dispatch.pushView(4);
      result.current.dispatch.popView();
    });
    expect(result.current.state.stackNavigator.top()).toBe(3);
  });

  test('`resetView` resets elements from stackNavigator', async () => {
    act(() => {
      result.current.dispatch.pushView(3);
      result.current.dispatch.pushView(4);
      result.current.dispatch.pushView(5);
      result.current.dispatch.resetView();
    });
    expect(result.current.state.stackNavigator.top()).toBe(0);
  });
});
