import { Meta, Story } from '@storybook/react';
import React from 'react';
import { ScheduledTaskTableProps, ScheduledTaskTable, ScheduledTask } from './scheduled-task-table';

export default {
  title: 'Tasks/Scheduled task table',
  component: ScheduledTaskTable,
} as Meta;

function makeTasks(): ScheduledTask[] {
  const tasks = [];
  for (let i = 0; i < 100; i++) {
    tasks.push({
      id: i,
      enabled: true,
      task_type: 'delivery',
      task_datetime: '2021-09-23T15:11:03.979209+00:00',
      args: null,
    });
  }
  return tasks;
}

export const ScheduledTaskTableStory: Story<ScheduledTaskTableProps> = (args) => {
  return <ScheduledTaskTable {...args}></ScheduledTaskTable>;
};

ScheduledTaskTableStory.args = {
  scheduledTasks: makeTasks(),
};
