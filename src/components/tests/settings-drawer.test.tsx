import React from 'react';
import { createMount } from '@material-ui/core/test-utils';
import toJson from 'enzyme-to-json';
import { Radio, RadioGroup } from '@material-ui/core';

import SettingsDrawer from '../settings-drawer';
import {
  loadSettings,
  TrajectoryAnimation,
  Settings,
  TrajectoryDiameter,
  TrajectoryColor,
} from '../../settings';
import { SettingsContext } from '../app-contexts';

const mount = createMount();

describe('Settings Drawer', () => {
  let settings = loadSettings();
  let onSettingsChange: jest.Mock;

  beforeEach(() => {
    onSettingsChange = jest.fn();
  });

  it('should render correctly', () => {
    const wrapper = mount(
      <SettingsContext.Provider value={settings}>
        <SettingsDrawer settings={settings} onSettingsChange={onSettingsChange} open={true} />
      </SettingsContext.Provider>,
    );
    expect(toJson(wrapper)).toMatchSnapshot();
  });

  it('should call onSettingsChange function when handleTrajectoryAnimationChange is called', () => {
    const newSettings: Settings = {
      ...settings,
      trajectoryAnimation: TrajectoryAnimation.Outline,
    };
    const mockEvent = { target: { value: TrajectoryAnimation.Outline } };
    const wrapper = mount(
      <SettingsContext.Provider value={newSettings}>
        <SettingsDrawer
          settings={settings}
          onSettingsChange={newSettings => onSettingsChange(newSettings)}
          open={true}
        />
      </SettingsContext.Provider>,
    );

    wrapper
      .find('input')
      .at(3)
      .simulate('change', mockEvent);
    expect(onSettingsChange).toHaveBeenCalledTimes(1);
  });

  it('should call onSettingsChange function when handleTrajectoryDiameterChange is called', () => {
    const newSettings: Settings = {
      ...settings,
      trajectoryDiameter: TrajectoryDiameter.FixSize,
    };
    const mockEvent = { target: { value: TrajectoryDiameter.FixSize } };
    const wrapper = mount(
      <SettingsContext.Provider value={newSettings}>
        <SettingsDrawer
          settings={settings}
          onSettingsChange={newSettings => onSettingsChange(newSettings)}
          open={true}
        />
      </SettingsContext.Provider>,
    );

    wrapper
      .find('input')
      .at(4)
      .simulate('change', mockEvent);
    expect(onSettingsChange).toHaveBeenCalledTimes(1);
  });

  it('should call onSettingsChange function when handleTrajectoryColorChange is called', () => {
    const newSettings: Settings = {
      ...settings,
      trajectoryColor: TrajectoryColor.Theme,
    };
    const mockEvent = { target: { value: TrajectoryColor.Theme } };
    const wrapper = mount(
      <SettingsContext.Provider value={newSettings}>
        <SettingsDrawer
          settings={settings}
          onSettingsChange={newSettings => onSettingsChange(newSettings)}
          open={true}
        />
      </SettingsContext.Provider>,
    );

    wrapper
      .find('input')
      .at(6)
      .simulate('change', mockEvent);
    expect(onSettingsChange).toHaveBeenCalledTimes(1);
  });
});
