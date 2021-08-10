import { Meta, Story } from '@storybook/react';
import React from 'react';
import { LiftPanel } from './lift-panel';
import { testLifts, testLiftStates } from './test-utils.spec';

export default {
  title: 'Lift Panel',
  component: LiftPanel,
} as Meta;

export const DoorSidePanel: Story = (args) => {
  return <LiftPanel lifts={testLifts} liftStates={testLiftStates} {...args} />;
};
