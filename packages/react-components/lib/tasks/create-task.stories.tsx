import { Meta, Story } from '@storybook/react';
import type { TaskRequest } from 'api-client';
import React from 'react';
import { CreateTaskForm, CreateTaskFormProps } from './create-task';
import { makeTaskRequest } from './test-data.spec';

export default {
  title: 'Tasks/Create Task',
  component: CreateTaskForm,
} as Meta;

function makeTasks(): TaskRequest[] {
  const tasks: TaskRequest[] = [];
  for (let i = 0; i < 100; i++) {
    tasks.push(makeTaskRequest());
  }
  return tasks;
}

export const CreateTask: Story<CreateTaskFormProps> = (args) => {
  return <CreateTaskForm {...args} open tasksFromFile={makeTasks}></CreateTaskForm>;
};

CreateTask.args = {
  submitTasks: async () => new Promise((res) => setTimeout(res, 500)),
  cleaningZones: ['test_zone_0', 'test_zone_1'],
  loopWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  deliveryWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  dispensers: ['test_dispenser_0', 'test_dispenser_1'],
  ingestors: ['test_ingestor_0', 'test_ingestor_1'],
};
