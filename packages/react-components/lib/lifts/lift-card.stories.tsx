import { CardActions } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { LiftState } from 'rmf-models';
import { LiftCard, LiftCardProps } from './lift-card';
import { LiftControls } from './lift-controls';

export default {
  title: 'Lift Card',
} satisfies Meta;

export const Default: StoryFn<LiftCardProps> = (args) => <LiftCard sx={{ width: 200 }} {...args} />;
Default.args = {
  name: 'main_lift',
  motionState: LiftState.MOTION_UP,
  currentFloor: 'L1',
  destinationFloor: 'L2',
  doorState: LiftState.DOOR_CLOSED,
};

export const WithControls: StoryFn<LiftCardProps> = (args) => (
  <LiftCard sx={{ width: 200 }} {...args}>
    <CardActions sx={{ justifyContent: 'center' }}>
      <LiftControls availableLevels={['L1', 'L2']} currentLevel={args.currentFloor || ''} />
    </CardActions>
  </LiftCard>
);
WithControls.args = Default.args;
