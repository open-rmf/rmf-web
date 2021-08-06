import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskTimeline, TaskTimelineProps } from './task-timeline';
import { makeTask } from './test-data.spec';
import * as RmfModels from 'rmf-models';

const task = makeTask('test_task', 3, 3);

export default {
  title: 'Tasks/Timeline',
  component: TaskTimeline,
} as Meta;

export const activeTimeline: Story<TaskTimelineProps> = (args) => {
  return <TaskTimeline {...args}></TaskTimeline>;
};
const activeTask = makeTask('active_task', 3, 3);
activeTask.state = RmfModels.TaskSummary.STATE_ACTIVE;

activeTimeline.args = {
  taskSummary: task,
};

export const pendingTimeline: Story<TaskTimelineProps> = (args) => {
  return <TaskTimeline {...args}></TaskTimeline>;
};

const pendingTask = makeTask('pending_task', 3, 3);
pendingTask.state = RmfModels.TaskSummary.STATE_PENDING;

pendingTimeline.args = {
  taskSummary: pendingTask,
};

export const completedTimeline: Story<TaskTimelineProps> = (args) => {
  return <TaskTimeline {...args}></TaskTimeline>;
};

const completedTask = makeTask('completed_task', 3, 3);
completedTask.state = RmfModels.TaskSummary.STATE_COMPLETED;

completedTimeline.args = {
  taskSummary: completedTask,
};

export const failedTimeline: Story<TaskTimelineProps> = (args) => {
  return <TaskTimeline {...args}></TaskTimeline>;
};

const failedTask = makeTask('failed_task', 3, 3);
failedTask.state = RmfModels.TaskSummary.STATE_FAILED;

failedTimeline.args = {
  taskSummary: failedTask,
};
