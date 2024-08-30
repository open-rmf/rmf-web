import { CardActions } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';
import { Door } from 'rmf-models/ros/rmf_building_map_msgs/msg';
import { DoorMode } from 'rmf-models/ros/rmf_door_msgs/msg';

import { DoorCard } from './door-card';
import { DoorControls } from './door-controls';

export default {
  title: 'Door Card',
  component: DoorCard,
} satisfies Meta;

type Story = StoryObj<typeof DoorCard>;

export const Default: Story = {
  args: {
    name: 'main_door',
    level: 'L1',
    mode: DoorMode.MODE_OPEN,
    type: Door.DOOR_TYPE_SINGLE_SWING,
  },
  render: (args) => <DoorCard sx={{ width: 200 }} {...args} />,
};

export const WithControls: Story = {
  args: Default.args,
  render: (args) => (
    <DoorCard sx={{ width: 200 }} {...args}>
      <CardActions sx={{ justifyContent: 'center' }}>
        <DoorControls />
      </CardActions>
    </DoorCard>
  ),
};
