import { Meta, StoryObj } from '@storybook/react';

import { TaskLogs } from './task-logs';
import { makeTaskLog } from './test-data.spec';

export default {
  title: 'Tasks/Logs',
  component: TaskLogs,
} satisfies Meta;

type Story = StoryObj<typeof TaskLogs>;

export const Logs: Story = {
  args: {
    taskLog: makeTaskLog('task'),
    eventName: (_phaseId, eventId) => `Event ${eventId}`,
    eventStatus: () => 'completed',
  },
};
