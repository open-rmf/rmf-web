import React from 'react';
import { createMount } from '@material-ui/core/test-utils';
import toJson from 'enzyme-to-json';

import SettingsDrawer from '../settings-drawer';
import { loadSettings } from '../../settings';

const mount = createMount();

describe('Settings Drawer', () => {
  let settings = loadSettings();

  it('should not crash on render', () => {
    const wrapper = mount(<SettingsDrawer settings={settings} onSettingsChange={jest.fn()} />);
    expect(toJson(wrapper)).toMatchSnapshot();
    wrapper.unmount();
  });
});
