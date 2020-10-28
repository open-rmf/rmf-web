import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { RobotAccordion } from '../lib';

export default {
  title: 'Robot Accordion',
  component: RobotAccordion,
  parameters: { actions: { argTypesRegex: '^on.*' } },
} as Meta;

const baseRobot: RomiCore.RobotState = {
  name: 'test',
  battery_percent: 1,
  location: { level_name: 'test_level', x: 0, y: 0, yaw: 0, t: { sec: 0, nanosec: 0 } },
  mode: { mode: RomiCore.RobotMode.MODE_PAUSED },
  model: 'test_model',
  task_id: 'test_task_id',
  path: [],
};

export const Basic: Story = (args) => (
  <RobotAccordion fleetName="test_fleet" robot={baseRobot} {...args} />
);
