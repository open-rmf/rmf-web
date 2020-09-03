import { createMount } from '@material-ui/core/test-utils';
import { LoopForm } from '../loop-form';
import React from 'react';
import { TLoopRequest } from '../commands-panel';
import { RobotResourceManager } from '../../resource-manager-robots';
import fakeResources from '../../mock/data/resources';

const mount = createMount();

const buildWrapper = (
  fleetName: string,
  onClick: TLoopRequest,
  robotResourceManager: RobotResourceManager,
) => {
  const wrapper = mount(
    <LoopForm requestLoop={onClick} fleetNames={[fleetName]} robotHandler={robotResourceManager} />,
  );
  return wrapper;
};

describe('Form validation', () => {
  const robotResourceManager = new RobotResourceManager(fakeResources().robots);
  let isRequestButtonClicked = false;
  const onClick = (
    fleetName: string,
    numLoops: number,
    startLocationPoint: string,
    endLocationPoint: string,
  ) => {
    isRequestButtonClicked = true;
  };

  beforeEach(() => {
    isRequestButtonClicked = false;
  });

  test('Initial Values', () => {
    const wrapper = buildWrapper('tinyRobot', onClick, robotResourceManager);
    expect(wrapper.find('input[name="targetFleet"]').props().value != '');
    expect(wrapper.find("input[type='number']").props().value).toEqual('');
    expect(wrapper.findWhere(x => x.name() === 'input' && x.props().value != '')).toBeTruthy();
    expect(wrapper.find('input[name="startLocation"]').props().value).not.toEqual(
      wrapper.find('input[name="finishLocation"]').props().value,
    );
    wrapper.unmount();
  });

  test('Successful Request', () => {
    const wrapper = buildWrapper('tinyRobot', onClick, robotResourceManager);
    wrapper.find("input[type='number']").simulate('change', { target: { value: 1 } });
    expect(wrapper.find("input[type='number']").props().value).toEqual(1);
    wrapper.find('form').simulate('submit');
    expect(isRequestButtonClicked).toBe(true);
  });

  test('Number of loops cannot be empty', async () => {
    const wrapper = buildWrapper('tinyRobot', onClick, robotResourceManager);
    wrapper.find('form').simulate('submit');
    expect(isRequestButtonClicked).toBeFalsy();
    expect(wrapper.exists('#numLoops-helper-text')).toBeTruthy();
    wrapper.unmount();
  });

  test('Location cannot be empty', async () => {
    const wrapper = buildWrapper('FleetA', onClick, robotResourceManager);
    wrapper.find("input[type='number']").simulate('change', { target: { value: 1 } });
    wrapper.find('form').simulate('submit');
    expect(isRequestButtonClicked).toBeFalsy();
    expect(wrapper.exists('#startLocation-helper-text')).toBeTruthy();
    wrapper.unmount();
  });

  test('Start Location cannot be equal to Finish Location', async () => {
    const wrapper = buildWrapper('FleetB', onClick, robotResourceManager);
    wrapper.find("input[type='number']").simulate('change', { target: { value: 1 } });
    wrapper.find('form').simulate('submit');
    expect(isRequestButtonClicked).toBeFalsy();
    expect(wrapper.exists('#startLocation-helper-text')).toBeTruthy();
    expect(wrapper.exists('#finishLocation-helper-text')).toBeTruthy();
    wrapper.unmount();
  });
});
