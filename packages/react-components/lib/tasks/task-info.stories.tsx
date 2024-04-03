import { Paper } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { TaskInfo, TaskInfoProps } from './task-info';
import { makeTaskState } from './test-data.spec';

export default {
  title: 'Tasks/Task Info',
  component: TaskInfo,
} satisfies Meta;

export const Default: StoryFn<TaskInfoProps> = (args) => {
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
