import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskLogs, TaskLogsProps } from './task-logs';
import { makeTaskLog } from './test-data.spec';

export default {
  title: 'Tasks/Logs',
  component: TaskLogs,
  argTypes: {
    paginationOptions: {
      control: {
        disable: true,
      },
    },
    submitTask: {
      control: {
        disable: true,
      },
    },
  },
} as Meta;

export const Logs: Story<TaskLogsProps> = (args) => {
  return <TaskLogs {...args} />;
};

Logs.args = {
  taskLog: makeTaskLog('task'),
};
