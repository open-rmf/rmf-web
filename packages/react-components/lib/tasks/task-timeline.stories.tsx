import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskTimeline, TaskTimelineProps } from './task-timeline';
import { TaskState } from 'api-client';

export default {
  title: 'Tasks/Timeline',
  component: TaskTimeline,
} as Meta;

export const Timeline: Story<TaskTimelineProps> = (args) => {
  return <TaskTimeline {...args}></TaskTimeline>;
};
const task: TaskState = {
  active: 1,
  booking: { id: 'Loop1', unix_millis_earliest_start_time: 7856, priority: null, labels: null },
  cancellation: null,
  category: 'Loop',
  completed: [],
  detail: {},
  estimate_millis: 1076180,
  interruptions: null,
  killed: null,
  pending: [2, 3],
  phases: {
    '1': {
      category: 'Go to [place:pantry]',
      detail: 'Moving the robot from [place:tinyRobot1_charger] to [place:pantry]',
      estimate_millis: 9597,
      events: null,
      final_event_id: 0,
      id: 1,
    },
    '2': {
      category: 'Go to [place:supplies]',
      detail: 'Moving the robot from [place:pantry] to [place:supplies]',
      estimate_millis: 9597,
      events: null,
      final_event_id: 0,
      id: 2,
    },
    '3': {
      category: 'Go to [place:pantry]',
      detail: 'Moving the robot from [place:supplies] to [place:pantry]',
      estimate_millis: 9597,
      events: null,
      final_event_id: 0,
      id: 3,
    },
  },
  unix_millis_finish_time: null,
  unix_millis_start_time: null,
};

Timeline.args = {
  taskState: task,
};
