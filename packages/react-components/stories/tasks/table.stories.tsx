import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskTable, TaskTableProps } from '../../lib';
import { makeTask } from './test-data';
import * as RmfModels from 'rmf-models';

const completedTask = makeTask('completed_task', 3, 3);
completedTask.state = RmfModels.TaskSummary.STATE_COMPLETED;
const failedTask = makeTask('failed_task', 3, 3);
failedTask.state = RmfModels.TaskSummary.STATE_FAILED;
const tasks = [
  makeTask('active_task', 3, 3),
  makeTask('active_task_2', 4, 3),
  completedTask,
  failedTask,
];

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
