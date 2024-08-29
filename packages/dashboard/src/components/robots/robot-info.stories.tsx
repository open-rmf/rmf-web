import { Meta, StoryObj } from '@storybook/react';

import { RobotInfo } from './robot-info';

export default {
  title: 'Robots/Detailed Info',
  component: RobotInfo,
} satisfies Meta;

type Story = StoryObj<typeof RobotInfo>;

export const Default: Story = {
  storyName: 'Detailed Info',
  args: {
    robotName: 'Robot Name',
    assignedTask: 'mytask',
    battery: 0.5,
    taskProgress: 0.5,
    taskStatus: 'underway',
    estFinishTime: Date.now(),
  },
};
