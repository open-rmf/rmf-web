import { act } from '@testing-library/react-hooks';
import { HotKeysEnabledProps } from 'react-hotkeys';
import { AppController } from '../components/app-contexts';
import { makeMockAppController } from '../components/tests/mock-app-controller';
import { buildHotKeys, keyMap } from '../hotkeys';

test('build hotkeys on the correct format', () => {
  const hotkeys = buildHotKeys({
    appController: makeMockAppController(),
  });
  if (!hotkeys.keyMap) throw new Error('An error has occurred building the hotkeys formats');
  expect(Object.keys(hotkeys.keyMap)).toEqual(Object.keys(keyMap));
});

describe('update states correctly', () => {
  let hotkeys: HotKeysEnabledProps;
  let appController: AppController;

  beforeEach(() => {
    appController = makeMockAppController();
    hotkeys = buildHotKeys({
      appController,
    });
  });

  test('toggles Setting Panel correctly', () => {
    act(() => {
      hotkeys.handlers?.OPEN_SETTINGS();
    });
    expect(appController.toggleSettings).toBeCalledTimes(1);

    act(() => {
      hotkeys.handlers?.OPEN_SETTINGS();
    });
    expect(appController.toggleSettings).toBeCalledTimes(2);
  });
});
