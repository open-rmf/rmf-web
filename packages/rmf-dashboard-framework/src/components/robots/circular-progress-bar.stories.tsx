import { Typography } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';

import { CircularProgressBar } from './circular-progress-bar';

export default {
  title: 'Robots/Circular Progress Bar',
  component: CircularProgressBar,
} satisfies Meta;

type Story = StoryObj<typeof CircularProgressBar>;

export const Default: Story = {
  args: {
    progress: 70,
    strokeColor: 'green',
  },
  render: (args) => (
    <CircularProgressBar {...args}>
      <Typography variant="h6">{args.progress}%</Typography>
    </CircularProgressBar>
  ),
};
