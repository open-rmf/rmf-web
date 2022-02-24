import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskTimeline, TaskTimelineProps } from './task-timeline';
import { makeTaskState } from './test-data.spec';

export default {
  title: 'Tasks/Timeline',
  component: TaskTimeline,
} as Meta;

export const Timeline: Story<TaskTimelineProps> = (args) => {
  return <TaskTimeline {...args}></TaskTimeline>;
};

Timeline.args = {
  taskState: makeTaskState('task'),
};
