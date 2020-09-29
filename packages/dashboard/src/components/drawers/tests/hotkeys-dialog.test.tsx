import { shallow } from 'enzyme';
import React from 'react';
import { GlobalHotKeys, KeySequence } from 'react-hotkeys';
import HotKeysDialog from '../hotkeys-dialog';

describe('Hotkeys Dialog', () => {
  test('Component renders correctly', () => {
    const keyMap = {
      OPEN_COMMANDS: {
        name: 'Open Commands',
        sequences: [{ sequence: 'shift+c', action: 'keypress' }],
      } as KeySequence,
    };

    const handlers = {
      OPEN_COMMANDS: () => jest.fn(),
    };

    const root = shallow(
      <GlobalHotKeys keyMap={keyMap} handlers={handlers}>
        <HotKeysDialog handleClose={() => jest.fn()} open={true} />
      </GlobalHotKeys>,
    );
    const hotKeyDialog = root.find(HotKeysDialog).dive();
    expect(hotKeyDialog.find('[detail-type="hotkeyDetail"]').length).toBe(
      Object.keys(keyMap).length,
    );
    expect(root.find(HotKeysDialog).dive()).toMatchSnapshot();
  });
});
