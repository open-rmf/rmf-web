import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskTable, TaskTableProps } from '../../lib';
import { makeTask } from './test-data';

const tasks = [makeTask('test_task', 3, 3), makeTask('test_task_2', 4, 3)];

export default {
  title: 'Tasks/Table',
  component: TaskTable,
} as Meta;

export const Table: Story<TaskTableProps> = (args) => {
  return <TaskTable {...args}></TaskTable>;
};

Table.args = {
  tasks: tasks,
};
