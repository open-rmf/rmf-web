import { Meta, Story } from '@storybook/react';
import React from 'react';
import { RobotInfo, RobotInfoProps } from '../../lib';
import { makeRobot } from '../../lib/robots/test-utils.spec';
import { makeDefinedTask } from '../../lib/tasks/test-data.spec';

export default {
  title: 'Robots/Info',
  component: RobotInfo,
} as Meta;

export const Info: Story<RobotInfoProps> = (args) => {
  return <RobotInfo {...args}></RobotInfo>;
};

const exampleRobot = makeRobot();
const tasks = [
  makeDefinedTask('Loop', 'test', '1002', 3, 1),
  makeDefinedTask('Delivery', 'test', '1004', 3, 1),
  makeDefinedTask('Loop', 'test', '1007', 4, 1),
];
const verboseRobot = {
  fleet: 'fleet',
  name: exampleRobot.name,
  state: exampleRobot,
  tasks,
};

Info.args = {
  robot: verboseRobot,
};
