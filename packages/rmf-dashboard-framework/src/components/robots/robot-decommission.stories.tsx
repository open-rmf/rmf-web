import { Meta, StoryObj } from '@storybook/react';

import { RobotDecommissionButton } from './robot-decommission';
import { makeRobot } from './test-utils.test';

export default {
  title: 'Robots/RobotDecommissionButton',
  component: RobotDecommissionButton,
} satisfies Meta;

type Story = StoryObj<typeof RobotDecommissionButton>;

export const Default: Story = {
  args: {
    fleet: 'test_fleet',
    robotState: makeRobot({ name: 'test_robot' }),
  },
};
