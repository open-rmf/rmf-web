import { CardActions } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { Door, DoorMode } from 'rmf-models';
import { DoorCard, DoorCardProps } from './door-card';
import { DoorControls } from './door-controls';

export default {
  title: 'Door Card',
} satisfies Meta;

export const Default: StoryFn<DoorCardProps> = (args) => <DoorCard sx={{ width: 200 }} {...args} />;
Default.args = {
  name: 'main_door',
  level: 'L1',
  mode: DoorMode.MODE_OPEN,
  type: Door.DOOR_TYPE_SINGLE_SWING,
};

export const WithControls: StoryFn<DoorCardProps> = (args) => (
  <DoorCard sx={{ width: 200 }} {...args}>
    <CardActions sx={{ justifyContent: 'center' }}>
      <DoorControls />
    </CardActions>
  </DoorCard>
);
WithControls.args = Default.args;
