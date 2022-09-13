import { Meta, Story } from '@storybook/react';
import React from 'react';
import { RobotIssues, RobotIssuesProps } from './robot-issues';
import { makeRobot } from './test-utils.spec';

export default {
  title: 'Robot Issues',
  component: RobotIssues,
} as Meta;

export const Default: Story<RobotIssuesProps> = (args) => {
  return <RobotIssues {...args}></RobotIssues>;
};

Default.storyName = 'Robot Issues';

Default.args = {
  robotIssues: makeRobot({ name: 'test_robot' }).issues,
};
