import { Meta, StoryObj } from '@storybook/react';

import { TaskForm } from './task-form';

export default {
  title: 'Tasks/Create Task',
  component: TaskForm,
} satisfies Meta;

type Story = StoryObj<typeof TaskForm>;

export const OpenTaskForm: Story = {
  args: {
    onDispatchTask: async () => new Promise((res) => setTimeout(res, 500)),
    onScheduleTask: async () => new Promise((res) => setTimeout(res, 500)),
    cleaningZones: ['test_zone_0', 'test_zone_1'],
    patrolWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
    pickupPoints: { test_waypoint_0: 'test_waypoint_0' },
    dropoffPoints: { test_waypoint_1: 'test_waypoint_1' },
  },
  render: (args) => {
    return <TaskForm {...args} open></TaskForm>;
  },
};
