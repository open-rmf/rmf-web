import { Meta, StoryObj } from '@storybook/react';
import { ApiServerModelsRmfApiRobotStateStatus as Status } from 'api-client';

import { RobotSummary } from './robot-summary';

export default {
  title: 'Robots/RobotSummary',
  component: RobotSummary,
} satisfies Meta;

type Story = StoryObj<typeof RobotSummary>;

export const Default: Story = {
  args: {
    onClose: () => {},
    robot: {
      fleet: 'test_fleet',
      name: 'test_robot',
      status: Status.Idle,
      battery: 60,
      estFinishTime: 1000000,
      lastUpdateTime: 900000,
      level: 'L1',
      commission: {
        dispatch_tasks: true,
        direct_tasks: true,
        idle_behavior: true,
      },
    },
  },
};
