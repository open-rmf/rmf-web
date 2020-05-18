import { createMount } from '@material-ui/core/test-utils';
import { RomiCoreLift } from '../lift-overlay';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import fakeLiftStates from '../../../mock/data/lift-states';
import getBuildingMap from '../../../mock/data/building-map';
import React from 'react';
import LiftContainer, { LiftModeStates, LiftMotionStates } from './liftContainer';
import Lift from './lift';

const mount = createMount();

const buildWrapper = (
  lift: RomiCoreLift,
  state: RomiCore.LiftState,
  currentFloor: string = 'L1',
) => {
  const wrapper = mount(
    <svg>
      <LiftContainer currentFloor={currentFloor} lift={lift} liftState={state} />
    </svg>,
  );

  const liftSVGRect = wrapper
    .find(LiftContainer)
    .at(0)
    .find('rect');

  return { wrapper, liftSVGRect };
};

it('Trigger click event', async () => {
  const buildingMap = await getBuildingMap();
  const lifts = buildingMap.lifts;
  const lift = lifts[0];
  let clicked = false;

  const wrapper = mount(
    <svg>
      <LiftContainer currentFloor={'L1'} lift={lift} onClick={() => (clicked = true)} />
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
  const loadLift = async () => {
    const buildingMap = await getBuildingMap();
    const lifts = buildingMap.lifts;
    const lift = lifts[0];
    const state = fakeLiftStates()[lift.name];
    return { lift, state };
  };

  test('Check red color and _Fire_ text when the lift its on Fire mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = LiftModeStates.FIRE;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(fire)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMode').text()).toEqual('FIRE!');
    wrapper.unmount();
  });

  test('Lift on Emergency mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = LiftModeStates.EMERGENCY;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(emergency)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMode').text()).toEqual('EMERGENCY!');
    wrapper.unmount();
  });

  test('Lift on Disabled mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = LiftModeStates.OFFLINE;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(offLine)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMode').text()).toEqual('OFFLINE');
    wrapper.unmount();
  });
});

describe('Checks assignation of styles on combination of motion states and mode of the Lift', () => {
  const loadLift = async () => {
    const buildingMap = await getBuildingMap();
    const lifts = buildingMap.lifts;
    const lift = lifts[0];
    const state = fakeLiftStates()[lift.name];
    return { lift, state };
  };

  test('Lift its on current floor and its on Human mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = LiftModeStates.HUMAN;
    state.current_floor = 'L1';
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(humanMode)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual('STOPPED');
    expect(wrapper.find('#liftMode').text()).toEqual('HUMAN');
    wrapper.unmount();
  });

  test('Lift its on current floor and its on SVG mode', async () => {
    const { lift, state } = await loadLift();
    state.current_mode = LiftModeStates.AGV;
    state.current_floor = 'L1';
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(liftOnCurrentFloor)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual('STOPPED');
    expect(wrapper.find('#liftMode').text()).toEqual('AGV');
    wrapper.unmount();
  });

  test('Lift its not on current floor and it`s going to an upper floor', async () => {
    const { lift, state } = await loadLift();
    state.current_floor = 'L3';
    state.destination_floor = 'L4';
    state.motion_state = LiftMotionStates.UP;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(liftOnAnotherFloor)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual('L3 → L4');
    wrapper.unmount();
  });

  test('Lift its not on current floor and it`s going to an lower floor', async () => {
    const { lift, state } = await loadLift();
    state.current_floor = 'L4';
    state.destination_floor = 'L2';
    state.motion_state = LiftMotionStates.DOWN;
    const { wrapper, liftSVGRect } = buildWrapper(lift, state);
    expect(liftSVGRect.hasClass(/(makeStyles)-(liftOnAnotherFloor)-(\d+)/)).toBe(true);
    expect(wrapper.find('#liftMotion').text()).toEqual(`L4 → L2`);
    wrapper.unmount();
  });
});
