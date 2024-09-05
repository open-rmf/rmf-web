import { Meta, StoryObj } from '@storybook/react';

import { CreateTaskForm } from './create-task';

export default {
  title: 'Tasks/Create Task',
  component: CreateTaskForm,
} satisfies Meta;

type Story = StoryObj<typeof CreateTaskForm>;

export const CreateTask: Story = {
  args: {
    onDispatchTask: async () => new Promise((res) => setTimeout(res, 500)),
    onScheduleTask: async () => new Promise((res) => setTimeout(res, 500)),
    cleaningZones: ['test_zone_0', 'test_zone_1'],
    patrolWaypoints: ['test_waypoint_0', 'test_waypoint_1'],
    pickupPoints: { test_waypoint_0: 'test_waypoint_0' },
    dropoffPoints: { test_waypoint_1: 'test_waypoint_1' },
  },
  render: (args) => {
    return <CreateTaskForm {...args} open></CreateTaskForm>;
  },
};
