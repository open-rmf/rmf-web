import { makeMockAppController } from '../components/tests/mock-app-controller';
import { buildHotKeys, keyMap } from '../hotkeys';

test('build hotkeys on the correct format', () => {
  const hotkeys = buildHotKeys({
    appController: makeMockAppController(),
  });
  if (!hotkeys.keyMap) throw new Error('An error has occurred building the hotkeys formats');
  expect(Object.keys(hotkeys.keyMap)).toEqual(Object.keys(keyMap));
});
