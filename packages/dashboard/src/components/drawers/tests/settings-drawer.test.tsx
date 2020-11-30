import React from 'react';
import { createMount } from '@material-ui/core/test-utils';
import toJson from 'enzyme-to-json';
import { shallow } from 'enzyme';

import { loadSettings, TrajectoryAnimation, Settings } from '../../../settings';
import { SettingsContext } from '../../app-contexts';
import SettingsDrawer from '../../drawers/settings-drawer';

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
      trajectoryAnimation: TrajectoryAnimation.None,
    };
    const mockEvent = { target: { value: TrajectoryAnimation.None } };
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

    // Accessing the 'fill' animation @ the fourth input field
    wrapper.find('input').find({ name: 'None' }).simulate('change', mockEvent);
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
