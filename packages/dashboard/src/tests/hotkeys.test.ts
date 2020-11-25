import { renderHook } from '@testing-library/react-hooks';
import { useMainMenu } from '../components/reducers/main-menu-reducer';
import { buildHotKeys, keyMap } from '../hotkeys';
import { loadSettings } from '../settings';

test('Build hotkeys on the correct format', () => {
  const mainMenuInitialValues = {
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
  const { result } = renderHook(() => useMainMenu(mainMenuInitialValues));
  const hotkeys = buildHotKeys({ reducerMainMenu: result.current });

  const { keyMap: hotKeyMap } = hotkeys;

  if (!hotKeyMap) throw new Error('An error has occurred building the hotkeys formats');
  expect(Object.keys(hotKeyMap)).toEqual(Object.keys(keyMap));
});
