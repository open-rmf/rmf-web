import { renderHook } from '@testing-library/react-hooks';
import { mainMenuInitialValues } from '../components/reducers/main-menu-reducer-initial-values';
import { useMainMenu } from '../components/reducers/main-menu-reducer';
import { buildHotKeys, keyMap } from '../hotkeys';

test('Build hotkeys on the correct format', () => {
  const { result } = renderHook(() => useMainMenu(mainMenuInitialValues));
  const hotkeys = buildHotKeys({ reducerMainMenu: result.current });

  const { keyMap: hotKeyMap } = hotkeys;

  if (!hotKeyMap) throw new Error('An error has occurred building the hotkeys formats');
  expect(Object.keys(hotKeyMap)).toEqual(Object.keys(keyMap));
});
