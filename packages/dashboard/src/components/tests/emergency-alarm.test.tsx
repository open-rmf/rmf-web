import { IconButton } from '@material-ui/core';
import { mount, shallow } from 'enzyme';
import React from 'react';
import { EmergencyAlarm } from '../emergency-alarm';

describe('Emergency alarm behavior', () => {
  let root: any;

  afterEach(() => {
    root.unmount();
  });

  test('Alarm renders correctly when is activated', () => {
    root = shallow(<EmergencyAlarm isActive={true}></EmergencyAlarm>);
    expect(root).toMatchSnapshot();
  });

  test('Alarm icon is not displayed when is no activated', () => {
    root = mount(<EmergencyAlarm isActive={null}></EmergencyAlarm>);
    expect(root.find(IconButton).exists()).toBe(false);
  });

  test('Alarm icon displays a red icon when is activated', () => {
    root = mount(<EmergencyAlarm isActive={true}></EmergencyAlarm>);
    expect(root.find(IconButton).prop('color')).toBe('secondary');
  });
});
