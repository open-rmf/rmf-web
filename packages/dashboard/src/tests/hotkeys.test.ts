import { buildHotKeys, keyMap } from '../hotkeys';

test('Build hotkeys on the correct format', () => {
  const hotkeys = buildHotKeys({
    openCommands: () => jest.fn(),
    openDispensers: () => jest.fn(),
    openDoors: () => jest.fn(),
    openHelpPanel: () => jest.fn(),
    openHotKeys: () => jest.fn(),
    openLifts: () => jest.fn(),
    openOnmiPanel: () => jest.fn(),
    openRobots: () => jest.fn(),
    openSettings: () => jest.fn(),
  });

  const { keyMap: hotKeyMap } = hotkeys;

  if (!hotKeyMap) throw new Error('An error has occurred building the hotkeys formats');
  expect(Object.keys(hotKeyMap)).toEqual(Object.keys(keyMap));
});
