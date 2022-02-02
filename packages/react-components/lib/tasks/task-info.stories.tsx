import { Paper } from '@mui/material';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskInfo, TaskInfoProps } from './task-info';
import { makeTaskState } from './test-data.spec';

export default {
  title: 'Tasks/Task Info',
  component: TaskInfo,
} as Meta;

export const Default: Story<TaskInfoProps> = (args) => {
  return (
    <Paper style={{ padding: '1rem' }}>
      <TaskInfo {...args}></TaskInfo>
    </Paper>
  );
};
Default.storyName = 'Task Info';
Default.args = {
  task: makeTaskState('task'),
};
