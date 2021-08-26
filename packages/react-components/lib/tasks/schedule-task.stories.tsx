import { Meta, Story } from '@storybook/react';
import type { SubmitTask } from 'api-client';
import React from 'react';
import { ScheduleTaskForm, ScheduleTaskFormProps } from './schedule-task';
import { makeSubmitTask } from './test-data.spec';

export default {
  title: 'Tasks/Schedule Task',
  component: ScheduleTaskForm,
} as Meta;

function makeTasks(): SubmitTask[] {
  const tasks = [];
  for (let i = 0; i < 100; i++) {
    tasks.push(makeSubmitTask());
  }
  return tasks;
}

export const CreateTask: Story<ScheduleTaskFormProps> = (args) => {
  return <ScheduleTaskForm {...args} open tasksFromFile={makeTasks}></ScheduleTaskForm>;
};

CreateTask.args = {
  submitTasks: async () => new Promise((res) => setTimeout(res, 500)),
  cleaningZones: ['test_zone_0', 'test_zone_1'],
  loopWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  deliveryWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  dispensers: ['test_dispenser_0', 'test_dispenser_1'],
  ingestors: ['test_ingestor_0', 'test_ingestor_1'],
};
