import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskTable, TaskTableProps } from '../../lib';
import { makeTask } from './test-data';

const task = makeTask('test_task', 3, 3);

export default {
  title: 'Tasks/Table',
  component: TaskTable,
} as Meta;

export const Table: Story<TaskTableProps> = (args) => {
  return <TaskTable {...args}></TaskTable>;
};

Table.args = {
  tasks: [task],
};
