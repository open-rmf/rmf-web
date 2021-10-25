import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskSummary as RmfTaskSummary } from 'rmf-models';
import { TaskTimeline, TaskTimelineProps } from './task-timeline';
import { makeTaskSummaryWithPhases } from './test-data.spec';

export default {
  title: 'Tasks/Timeline',
  component: TaskTimeline,
} as Meta;

export const Timeline: Story<TaskTimelineProps> = (args) => {
  return <TaskTimeline {...args}></TaskTimeline>;
};
const task = makeTaskSummaryWithPhases('test_task', 3, 3);
task.state = RmfTaskSummary.STATE_ACTIVE;

Timeline.args = {
  taskSummary: task,
};
