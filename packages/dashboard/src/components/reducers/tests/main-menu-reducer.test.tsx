import { act, HookResult, renderHook } from '@testing-library/react-hooks';
import { mainMenuInitialValues } from '../../dashboard';
import { ReducerMainMenuProps, useMainMenuReducer } from '../main-menu-reducer';

describe('main menu reducer update states correctly', () => {
  let result: HookResult<ReducerMainMenuProps>;
  beforeEach(() => {
    const hookResult = renderHook(() => useMainMenuReducer(mainMenuInitialValues));
    result = hookResult.result;
  });

  test('updates current view correctly', () => {
    act(() => {
      result.current.dispatch.setCurrentView(2);
    });
    expect(result.current.state.currentView).toBe(2);
  });

  test('updates showHelp state correctly', () => {
    act(() => {
      result.current.dispatch.setShowHelp(true);
    });
    expect(result.current.state.showHelp).toBe(true);
  });

  test('updates showHotkeysDialog correctly', () => {
    act(() => {
      result.current.dispatch.setShowHotkeysDialog(true);
    });
    expect(result.current.state.showHotkeysDialog).toBe(true);
  });

  test('updates showOmniPanel correctly', () => {
    act(() => {
      result.current.dispatch.setShowOmniPanel(false);
    });
    expect(result.current.state.showOmniPanel).toBe(false);
  });

  test('updates showSettings correctly', () => {
    act(() => {
      result.current.dispatch.setShowSettings(true);
    });
    expect(result.current.state.showSettings).toBe(true);
  });

  test('updates tourState correctly', () => {
    act(() => {
      result.current.dispatch.setTourState(true);
    });
    expect(result.current.state.tourState).toBe(true);
  });

  test('toggles showHelp correctly', () => {
    act(() => {
      result.current.dispatch.toggleHelp();
    });
    expect(result.current.state.showHelp).toBe(true);
  });

  test('toggles toggleHotkeys correctly', () => {
    act(() => {
      result.current.dispatch.toggleHotkeys();
    });
    expect(result.current.state.showHotkeysDialog).toBe(true);
  });

  test('toggles showOmniPanel correctly', () => {
    act(() => {
      result.current.dispatch.toggleOmnipanel();
    });
    expect(result.current.state.showOmniPanel).toBe(false);
  });

  test('toggles showSettings correctly', () => {
    act(() => {
      result.current.dispatch.toggleSettings();
    });
    expect(result.current.state.showSettings).toBe(true);
  });

  test('`pushView` updates stackNavigator and currentView correctly', () => {
    act(() => {
      result.current.dispatch.pushView(3);
    });
    expect(result.current.state.currentView).toBe(3);
    expect(result.current.state.stackNavigator.top()).toBe(3);
  });

  test('`popView` pops elements from stackNavigator', () => {
    act(() => {
      result.current.dispatch.pushView(3);
      result.current.dispatch.pushView(4);
      result.current.dispatch.popView();
    });
    expect(result.current.state.stackNavigator.top()).toBe(3);
  });

  test('`resetView` resets elements from stackNavigator', () => {
    act(() => {
      result.current.dispatch.pushView(3);
      result.current.dispatch.pushView(4);
      result.current.dispatch.pushView(5);
      result.current.dispatch.resetView();
    });
    expect(result.current.state.stackNavigator.top()).toBe(0);
  });
});
