import React from 'react';
import { SystemSummaryTaskState, SystemSummaryTaskStateProps } from '../lib';
import { Meta, Story } from '@storybook/react';

export default {
  title: 'Systems summary task state',
  component: SystemSummaryTaskState,
} as Meta;

const systemSummaryTaskStateData: SystemSummaryTaskStateProps = {
  tasks: [
    {
      task_id: 'abc',
      state: 0,
      start_time: { sec: 0, nanosec: 0 },
      end_time: { sec: 0, nanosec: 0 },
      status: 'status',
      submission_time: { sec: 0, nanosec: 0 },
    },
    {
      task_id: 'abc',
      state: 1,
      start_time: { sec: 0, nanosec: 0 },
      end_time: { sec: 0, nanosec: 0 },
      status: 'status',
      submission_time: { sec: 0, nanosec: 0 },
    },
    {
      task_id: 'abc',
      state: 2,
      start_time: { sec: 0, nanosec: 0 },
      end_time: { sec: 0, nanosec: 0 },
      status: 'status',
      submission_time: { sec: 0, nanosec: 0 },
    },
    {
      task_id: 'abc',
      state: 3,
      start_time: { sec: 0, nanosec: 0 },
      end_time: { sec: 0, nanosec: 0 },
      status: 'status',
      submission_time: { sec: 0, nanosec: 0 },
    },
  ],
  onClick: () => {
    /**filler */
  },
};

export const SystemSummaryTaskStateStory: Story = (args) => (
  <SystemSummaryTaskState
    tasks={systemSummaryTaskStateData.tasks}
    onClick={systemSummaryTaskStateData.onClick}
    {...args}
  />
);
