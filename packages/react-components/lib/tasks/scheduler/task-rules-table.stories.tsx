import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskRuleTableProps, TaskRuleTable, TaskRule } from './task-rules-table';

export default {
  title: 'Tasks/Scheduled task table',
  component: TaskRuleTable,
} as Meta;

function makeTasks(): TaskRule[] {
  const tasks = [];
  for (let i = 0; i < 100; i++) {
    tasks.push({
      id: i,
      name: 'test' + i.toString(),
      task_type: 'delivery',
      frequency: 1,
      frequency_type: 'Once',
      first_day_to_apply_rule: null,
      created_at: '2021-08-30T16:14:25.955648+00:00',
      start_datetime: '2021-09-23T15:11:03.979209+00:00',
      end_datetime: null,
      days_of_week: null,
    });
  }
  return tasks;
}

export const TaskRuleTableStory: Story<TaskRuleTableProps> = (args) => {
  return <TaskRuleTable {...args}></TaskRuleTable>;
};

TaskRuleTableStory.args = {
  taskRules: makeTasks(),
};
