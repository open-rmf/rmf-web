import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import fakeLiftStates from '../../../mock/data/lift-states';
import getBuildingMap from '../../../mock/data/building-map';
import Lift from '../lift';
import React from 'react';
import toJson from 'enzyme-to-json';

const mount = createMount();

const getLiftComponent = (
  lift: RomiCore.Lift,
  state: RomiCore.LiftState,
  currentFloor: string = 'L1',
): React.ReactElement => {
  return (
    <svg>
      <Lift currentFloor={currentFloor} lift={lift} liftState={state} />
    </svg>
  );
};

const buildWrapper = (
  lift: RomiCore.Lift,
  state: RomiCore.LiftState,
  currentFloor: string = 'L1',
) => {
  const wrapper = mount(getLiftComponent(lift, state, currentFloor));

  const liftSVGRect = wrapper
    .find(Lift)
    .at(0)
    .find('rect');

  return { wrapper, liftSVGRect };
};

const loadLift = async () => {
  const buildingMap = await getBuildingMap();
  const lifts = buildingMap.lifts;
  const lift = lifts[0];
  const state = fakeLiftStates()[lift.name];
  return { lift, state };
};

it('Trigger click event', async () => {
  const buildingMap = await getBuildingMap();
  const lifts = buildingMap.lifts;
  const lift = lifts[0];
  let clicked = false;

  const wrapper = mount(
    <svg>
      <Lift currentFloor={'L1'} lift={lift} onClick={() => (clicked = true)} />
    </svg>,
  );

  wrapper
    .find(Lift)
    .at(0)
    .simulate('click');
  expect(clicked).toBe(true);

  wrapper.unmount();
});

describe('Checks assignation of styles on different mode of the Lift', () => {
  test('Check orange color and _Fire_ text when the lift its on Fire mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = RomiCore.LiftState.MODE_FIRE;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(fire)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMode').text()).toEqual('FIRE!');
    wrapper.unmount();
  });

  test('Lift on Emergency mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = RomiCore.LiftState.MODE_EMERGENCY;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(emergency)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMode').text()).toEqual('EMERGENCY!');
    wrapper.unmount();
  });

  test('Lift on Disabled mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = RomiCore.LiftState.MODE_OFFLINE;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(offLine)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMode').text()).toEqual('OFFLINE');
    wrapper.unmount();
  });

  test('Lift on Unknown mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = RomiCore.LiftState.MODE_UNKNOWN;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(unknownLift)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMode').exists()).toBe(false);
    wrapper.unmount();
  });
});

describe('Checks assignation of styles on combination of motion states and mode of the Lift', () => {
  test('Lift its on current floor and its on Human mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = RomiCore.LiftState.MODE_HUMAN;
    state.current_floor = 'L1';
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(humanMode)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual('L1');
    expect(wrapper.find('#liftMode').exists()).toBe(false);
    wrapper.unmount();
  });

  test('Lift its on current floor and its on SVG mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = RomiCore.LiftState.MODE_AGV;
    state.current_floor = 'L1';
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(liftOnCurrentFloor)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual('L1');
    expect(wrapper.find('#liftMode').exists()).toBe(false);
    wrapper.unmount();
  });

  test('Lift its not on current floor and it`s going to an upper floor', async () => {
    const { lift, state } = await loadLift();
    state.current_floor = 'L3';
    state.destination_floor = 'L4';
    state.motion_state = RomiCore.LiftState.MOTION_UP;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(liftMoving)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual('L3 → L4');
    expect(wrapper.find('#liftMode').exists()).toBe(false);
    wrapper.unmount();
  });

  test('Lift its not on current floor and it`s going to an lower floor', async () => {
    const { lift, state } = await loadLift();
    state.current_floor = 'L4';
    state.destination_floor = 'L2';
    state.motion_state = RomiCore.LiftState.MOTION_DOWN;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(liftMoving)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual(`L4 → L2`);
    expect(wrapper.find('#liftMode').exists()).toBe(false);
    wrapper.unmount();
  });

  test('Lift its not on current floor and its mode is UNKNOWN', async () => {
    const { lift, state } = await loadLift();
    state.current_floor = 'L4';
    state.destination_floor = 'L2';
    state.current_mode = RomiCore.LiftState.MODE_HUMAN;
    state.motion_state = RomiCore.LiftState.MOTION_DOWN;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(liftMoving)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual(`L4 → L2`);
    expect(wrapper.find('#liftMode').exists()).toBe(false);
    wrapper.unmount();
  });

  test('Lift its on current floor and its mode is UNKNOWN', async () => {
    const { lift, state } = await loadLift();
    state.current_floor = 'L1';
    state.current_mode = RomiCore.LiftState.MODE_UNKNOWN;
    state.motion_state = RomiCore.LiftState.MOTION_STOPPED;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(unknownLift)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual(`L1`);
    expect(wrapper.find('#liftMode').exists()).toBe(false);
    wrapper.unmount();
  });
});

test('Lift renders correctly', async () => {
  const { lift, state } = await loadLift();
  state.current_floor = 'L1';
  state.current_mode = RomiCore.LiftState.MODE_FIRE;
  state.motion_state = RomiCore.LiftState.MOTION_STOPPED;
  const { wrapper } = buildWrapper(lift, state);
  expect(toJson(wrapper)).toMatchSnapshot();
});
