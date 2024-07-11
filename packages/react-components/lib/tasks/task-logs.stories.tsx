import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { TaskLogs, TaskLogsProps } from './task-logs';
import { makeTaskLog } from './test-data.spec';

export default {
  title: 'Tasks/Logs',
  component: TaskLogs,
} satisfies Meta;

export const Logs: StoryFn<TaskLogsProps> = (args) => {
  return <TaskLogs {...args} />;
};

const taskLog = makeTaskLog('task');

Logs.args = {
  taskLog,
  eventName: (_phaseId, eventId) => `Event ${eventId}`,
  eventStatus: () => 'completed',
};
