import { Meta, Story } from '@storybook/react';
import React from 'react';
import { makeVerboseRobot, RobotInfo, RobotInfoProps } from '../../lib';
import { makeRobot } from '../../tests/robots/test-utils';
import { makeDefinedTask } from '../../tests/test-data/tasks';

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
const verboseRobot = makeVerboseRobot(exampleRobot, tasks);

Info.args = {
  robot: verboseRobot,
};
