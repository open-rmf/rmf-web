import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskTimeline, TaskTimelineProps } from './task-timeline';
import { makeTask } from './test-data.spec';
import * as RmfModels from 'rmf-models';

export default {
  title: 'Tasks/Timeline',
  component: TaskTimeline,
} as Meta;

export const Timeline: Story<TaskTimelineProps> = (args) => {
  return <TaskTimeline {...args}></TaskTimeline>;
};
const task = makeTask('test_task', 3, 3);
task.state = RmfModels.TaskSummary.STATE_ACTIVE;

Timeline.args = {
  taskSummary: task,
};
