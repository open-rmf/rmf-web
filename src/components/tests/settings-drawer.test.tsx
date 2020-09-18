import React from 'react';
import { createMount } from '@material-ui/core/test-utils';
import toJson from 'enzyme-to-json';
import { shallow } from 'enzyme';

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
  let handleCloseButton: jest.Mock;

  beforeEach(() => {
    onSettingsChange = jest.fn();
    handleCloseButton = jest.fn();
  });

  it('should render correctly', () => {
    const wrapper = shallow(
      <SettingsContext.Provider value={settings}>
        <SettingsDrawer
          handleCloseButton={handleCloseButton}
          settings={settings}
          onSettingsChange={onSettingsChange}
          open={true}
        />
      </SettingsContext.Provider>,
    );
    expect(toJson(wrapper.dive())).toMatchSnapshot();
    wrapper.unmount();
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
          onSettingsChange={(newSettings) => onSettingsChange(newSettings)}
          open={true}
          handleCloseButton={handleCloseButton}
        />
      </SettingsContext.Provider>,
    );

    // Accessing the 'outline' animation @ the fourth input field
    wrapper.find('input').find({ name: 'Outline' }).simulate('change', mockEvent);
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
          onSettingsChange={(newSettings) => onSettingsChange(newSettings)}
          open={true}
          handleCloseButton={handleCloseButton}
        />
      </SettingsContext.Provider>,
    );

    // Accessing the 'FixSize' animation @ the fifth input field
    wrapper.find('input').find({ name: 'Fix Size' }).simulate('change', mockEvent);
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
          onSettingsChange={(newSettings) => onSettingsChange(newSettings)}
          open={true}
          handleCloseButton={handleCloseButton}
        />
      </SettingsContext.Provider>,
    );

    // Accessing the 'Theme' animation @ the seventh input field
    wrapper.find('input').find({ name: 'Theme' }).simulate('change', mockEvent);
    expect(onSettingsChange).toHaveBeenCalledTimes(1);
  });

  it('should call handleCloseButton function when closeIcon button is clicked', () => {
    const wrapper = mount(
      <SettingsContext.Provider value={settings}>
        <SettingsDrawer
          settings={settings}
          onSettingsChange={(newSettings) => onSettingsChange(newSettings)}
          open={true}
          handleCloseButton={handleCloseButton}
        />
      </SettingsContext.Provider>,
    );
    wrapper.find('#closeDrawerButton').find('button').simulate('click');
    expect(handleCloseButton).toHaveBeenCalledTimes(1);
  });
});
