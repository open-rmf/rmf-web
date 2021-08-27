import { Meta, Story } from '@storybook/react';
import React from 'react';
import { ScheduleTaskForm, ScheduleTaskFormProps } from './schedule-task';

export default {
  title: 'Tasks/Schedule Task',
  component: ScheduleTaskForm,
} as Meta;

export const ScheduleTask: Story<ScheduleTaskFormProps> = (args) => {
  return <ScheduleTaskForm {...args} open></ScheduleTaskForm>;
};

ScheduleTask.args = {
  submitTask: async () => new Promise((res) => setTimeout(res, 500)),
  cleaningZones: ['test_zone_0', 'test_zone_1'],
  loopWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  deliveryWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
  dispensers: ['test_dispenser_0', 'test_dispenser_1'],
  ingestors: ['test_ingestor_0', 'test_ingestor_1'],
};
