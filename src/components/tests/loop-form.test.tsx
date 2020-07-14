import { createMount } from '@material-ui/core/test-utils';
import { LoopForm } from '../loop-form';
import fakePlaces from '../../mock/data/places';
import React from 'react';

const mount = createMount();

const buildWrapper = (fleetName: string) => {
  const onClick = (
    fleetName: string,
    numLoops: number,
    startLocationPoint: string,
    endLocationPoint: string,
  ) => {
    console.log('test');
  };

  const wrapper = mount(<LoopForm requestLoop={onClick} fleets={[fleetName]} />);
  return wrapper;
};

describe('form Validation', () => {
  test('Initial Values', () => {
    const wrapper = buildWrapper('SuperFleet');
    expect(wrapper.find('input[name="targetFleet"]').props().value != "");
    expect(wrapper.find("input[type='number']").props().value).toEqual('');
    expect(wrapper.findWhere(x => x.name() === 'input' && x.props().value != '')).toBeTruthy();
    expect(wrapper.find('input[name="startLocation"]').props().value).not.toEqual(
      wrapper.find('input[name="finishLocation"]').props().value,
    );
    wrapper.unmount();
  });

  test('Number of loops cannot be empty', async () => {
    const spy = jest.spyOn(console, 'log');
    const wrapper = buildWrapper('SuperFleet');
    wrapper.find('form').simulate('submit');
    expect(spy).not.toBeCalled();
    expect(wrapper.exists('.Mui-error')).toBeTruthy();
    wrapper.unmount();
  });

  test('Location cannot be empty', async () => {
    const spy = jest.spyOn(console, 'log');
    const wrapper = buildWrapper('FleetA');
    wrapper.find('form').simulate('submit');
    expect(spy).not.toBeCalled();
    expect(wrapper.exists('.Mui-error')).toBeTruthy();
    wrapper.unmount();
  });

  test('Start Location cannot be equal to Finish Location', async () => {
    const spy = jest.spyOn(console, 'log');
    const wrapper = buildWrapper('FleetB');
    wrapper.find('form').simulate('submit');
    expect(spy).not.toBeCalled();
    expect(wrapper.exists('.Mui-error')).toBeTruthy();
    wrapper.unmount();
  });
});
