import { Meta, Story } from '@storybook/react';
import { TaskProgress } from 'api-client';
import React from 'react';
import { RobotPanel as RobotPanel_, RobotPanelProps } from '../../lib';
import { makeRandomRobot } from '../../tests/robots/test-utils';
import { makeDefinedTask } from '../../tests/test-data/tasks';

export default {
  title: 'Robots/Robot Panel',
  component: RobotPanel_,
  argTypes: {
    fetchTasks: {
      table: {
        disable: true,
      },
    },
  },
} as Meta;

export const RobotPanel: Story<RobotPanelProps> = (args) => {
  return (
    <>
      <RobotPanel_
        {...args}
        style={{ height: '95vh', margin: 'auto', maxWidth: 1600 }}
      ></RobotPanel_>
    </>
  );
};

const robots = [
  makeRandomRobot('test_robot1', 'test_fleet', 2),
  makeRandomRobot('test_robot2', 'test_fleet', 1),
  makeRandomRobot('test_robot3', 'test_fleet', 3),
  makeRandomRobot('test_robot4', 'test_fleet', 4),
];

const tasks = [
  makeDefinedTask('Delivery', 'test_robot1', 'active_task_1', 3, 3),
  makeDefinedTask('Loop', 'test_robot2', 'active_task_2', 4, 3),
  makeDefinedTask('Clean', 'test_robot3', 'active_task_3', 4, 3),
  makeDefinedTask('Loop', 'test_robot4', 'active_task_4', 4, 3),
];

async function fetchTasks(): Promise<TaskProgress[]> {
  return tasks;
}

RobotPanel.args = {
  fetchTasks,
  robots,
};
