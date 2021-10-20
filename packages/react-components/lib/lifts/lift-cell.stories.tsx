import { Meta, Story } from '@storybook/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { LiftCell, LiftCellProps } from './lift-cell';
import { makeLift } from './test-utils.spec';

export default {
  title: 'Lift Cell',
} as Meta;

export const Default: Story<
  Pick<LiftCellProps, 'doorState' | 'motionState' | 'destinationFloor' | 'currentFloor'>
> = (args) => {
  return <LiftCell lift={makeLift({ name: 'example_lift' })} {...args} />;
};

Default.storyName = 'Lift Cell';

Default.args = {
  doorState: RmfModels.LiftState.DOOR_CLOSED,
  motionState: RmfModels.LiftState.MOTION_STOPPED,
  destinationFloor: 'L1',
  currentFloor: 'L1',
};
