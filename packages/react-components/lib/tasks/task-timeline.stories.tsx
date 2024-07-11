import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { TaskTimeline, TaskTimelineProps } from './task-timeline';
import { makeTaskState } from './test-data.spec';

export default {
  title: 'Tasks/Timeline',
  component: TaskTimeline,
} satisfies Meta;

export const Timeline: StoryFn<TaskTimelineProps> = (args) => {
  return <TaskTimeline {...args}></TaskTimeline>;
};

Timeline.args = {
  taskState: makeTaskState('task'),
};
