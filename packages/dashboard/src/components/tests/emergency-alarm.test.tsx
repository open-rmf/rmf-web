import { mount, shallow } from 'enzyme';
import React from 'react';
import { EmergencyAlarm } from '../emergency-alarm';
import { IconButton } from '@material-ui/core';
/**
 * We can't test alters from sweetalert2 here, so the complete functionality is tested
 * in an e2e test.
 */
describe('Emergency Alarm behavior', () => {
  let root: any;

  afterEach(() => {
    root.unmount();
  });

  test('Renders correctly', () => {
    root = shallow(<EmergencyAlarm isActive={null}></EmergencyAlarm>);
    expect(root).toMatchSnapshot();
  });

  test('Alarm shows a inherit color Icon when is deactivated', () => {
    root = mount(<EmergencyAlarm isActive={null}></EmergencyAlarm>);
    expect(root.find(IconButton).prop('color')).toBe('inherit');
  });

  test('Alarm shows a red icon when is activated', () => {
    root = mount(<EmergencyAlarm isActive={true}></EmergencyAlarm>);
    expect(root.find(IconButton).prop('color')).toBe('secondary');
  });
});
