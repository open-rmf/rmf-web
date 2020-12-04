import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskSummaryAccordion } from '../lib';

const tasks = [
  {
    end_time: { sec: 0, nanosec: 0 },
    start_time: { sec: 0, nanosec: 0 },
    state: 1,
    status:
      'Moving [tinyRobot/tinyRobot1]: ( 9.81228 -6.98942 -3.12904) -> ( 6.26403 -3.51569  1.16864) | Remaining phases: 1 | Remaining phases: 6',
    submission_time: { sec: 0, nanosec: 500 },
    task_id: '8b49e999-d246-4395-80f7-ebc5ca85e639',
  },
  {
    end_time: { sec: 0, nanosec: 0 },
    start_time: { sec: 0, nanosec: 0 },
    state: 1,
    status:
      'Moving [tinyRobot/tinyRobot2]: ( 9.81228 -6.98942 -3.12904) -> ( 6.26403 -3.51569  1.16864) | Remaining phases: 1 | Remaining phases: 6',
    submission_time: { sec: 0, nanosec: 1000 },
    task_id: '82e73d4f-1da7-474a-91ce-34cc92532455',
  },
];

export default {
  title: 'Task Summary Accordion',
  component: TaskSummaryAccordion,
  parameters: { actions: { argTypesRegex: '^on.*' } },
} as Meta;

export const TaskAccordion: Story = (args) => {
  return <TaskSummaryAccordion tasks={tasks} {...args} />;
};
