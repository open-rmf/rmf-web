import { Meta, Story } from '@storybook/react';
import React from 'react';
import { RobotInfo, RobotInfoProps } from './robot-info';

export default {
  title: 'Robots/Detailed Info',
  component: RobotInfo,
} as Meta;

export const Default: Story<RobotInfoProps> = (args) => {
  return <RobotInfo {...args}></RobotInfo>;
};

Default.storyName = 'Detailed Info';

Default.args = {
  robotName: 'Robot Name',
  assignedTask: 'mytask',
  battery: 0.5,
  taskProgress: 0.5,
  taskStatus: 'underway',
  estFinishTime: Date.now(),
};
