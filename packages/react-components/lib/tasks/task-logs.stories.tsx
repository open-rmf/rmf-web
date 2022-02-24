import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskLogs, TaskLogsProps } from './task-logs';
import { makeTaskLog } from './test-data.spec';

export default {
  title: 'Tasks/Logs',
  component: TaskLogs,
} as Meta;

export const Logs: Story<TaskLogsProps> = (args) => {
  return <TaskLogs {...args} />;
};

const taskLog = makeTaskLog('task');

Logs.args = {
  taskLog,
  eventName: (_phaseId, eventId) => `Event ${eventId}`,
  eventStatus: () => 'completed',
};
