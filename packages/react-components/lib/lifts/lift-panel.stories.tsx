import { Meta, Story } from '@storybook/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { LiftPanel } from './lift-panel';
import { makeLift, makeLiftState } from './test-utils.spec';

export default {
  title: 'Lift Panel',
  component: LiftPanel,
} as Meta;

const defaultLift = makeLift();
const lifts = [
  { ...defaultLift },
  { ...defaultLift, name: 'test1' },
  { ...defaultLift, name: 'test2' },
  { ...defaultLift, name: 'test3' },
];
const defaultState = makeLiftState();
const liftStates: Record<string, RmfModels.LiftState> = {
  test: defaultState,
  test1: {
    ...defaultState,
    door_state: RmfModels.LiftState.DOOR_MOVING,
    motion_state: RmfModels.LiftState.MOTION_DOWN,
  },
  test2: {
    ...defaultState,
    door_state: RmfModels.LiftState.DOOR_OPEN,
    motion_state: RmfModels.LiftState.MOTION_UP,
  },
  test3: defaultState,
};

export const DoorSidePanel: Story = (args) => {
  return <LiftPanel lifts={lifts} liftStates={liftStates} {...args} />;
};
