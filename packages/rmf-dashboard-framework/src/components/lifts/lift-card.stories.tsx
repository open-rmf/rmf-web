import { CardActions } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';
import { LiftState } from 'rmf-models/ros/rmf_lift_msgs/msg';

import { LiftCard } from './lift-card';
import { LiftControls } from './lift-controls';

export default {
  title: 'Lift Card',
  component: LiftCard,
} satisfies Meta;

type Story = StoryObj<typeof LiftCard>;

export const Default: Story = {
  args: {
    name: 'main_lift',
    motionState: LiftState.MOTION_UP,
    currentFloor: 'L1',
    destinationFloor: 'L2',
    doorState: LiftState.DOOR_CLOSED,
  },
  render: (args) => <LiftCard sx={{ width: 200 }} {...args} />,
};

export const WithControls: Story = {
  args: Default.args,
  render: (args) => (
    <LiftCard sx={{ width: 200 }} {...args}>
      <CardActions sx={{ justifyContent: 'center' }}>
        <LiftControls availableLevels={['L1', 'L2']} currentLevel={args.currentFloor || ''} />
      </CardActions>
    </LiftCard>
  ),
};
