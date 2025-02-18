import { Paper } from '@mui/material';
import { Meta, StoryObj } from '@storybook/react';

import { TaskInfo } from './task-info';
import { makeTaskState } from './test-data.test';

export default {
  title: 'Tasks/Task Info',
  component: TaskInfo,
} satisfies Meta;

type Story = StoryObj<typeof TaskInfo>;

export const Default: Story = {
  name: 'Task Info',
  args: {
    task: makeTaskState('task'),
  },
  render: (args) => (
    <Paper style={{ padding: '1rem' }}>
      <TaskInfo {...args}></TaskInfo>
    </Paper>
  ),
};
