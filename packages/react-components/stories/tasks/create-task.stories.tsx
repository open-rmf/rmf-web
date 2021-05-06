import { Meta, Story } from '@storybook/react';
import React from 'react';
import { CreateTaskForm, CreateTaskFormProps } from '../../lib';

export default {
  title: 'Tasks/Create Task',
  component: CreateTaskForm,
} as Meta;

export const CreateTask: Story<CreateTaskFormProps> = (args) => {
  return <CreateTaskForm {...args} open></CreateTaskForm>;
};

CreateTask.args = {
  submitTask: async () => new Promise((res) => setTimeout(res, 1000)),
  cleaningZones: ['test_zone_0', 'test_zone_1'],
  loopWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  deliveryWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  dispensers: ['test_dispenser_0', 'test_dispenser_1'],
  ingestors: ['test_ingestor_0', 'test_ingestor_1'],
};
