import { Meta, StoryFn } from '@storybook/react';
import type { TaskRequest } from 'api-client';
import React from 'react';
import { CreateTaskForm, CreateTaskFormProps } from './create-task';
import { makeTaskRequest } from './test-data.spec';

export default {
  title: 'Tasks/Create Task',
  component: CreateTaskForm,
} satisfies Meta;

export const CreateTask: StoryFn<CreateTaskFormProps> = (args) => {
  return <CreateTaskForm {...args} open></CreateTaskForm>;
};

CreateTask.args = {
  submitTasks: async () => new Promise((res) => setTimeout(res, 500)),
  cleaningZones: ['test_zone_0', 'test_zone_1'],
  patrolWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  pickupPoints: { test_waypoint_0: 'test_waypoint_0' },
  dropoffPoints: { test_waypoint_1: 'test_waypoint_1' },
};
