import { Meta, StoryObj } from '@storybook/react';

import { makeDefaultPatrolTaskDescription, PatrolTaskForm } from './patrol';

export default {
  title: 'Tasks/PatrolTaskForm',
  component: PatrolTaskForm,
} satisfies Meta;

type Story = StoryObj<typeof PatrolTaskForm>;

export const Default: Story = {
  args: {
    taskDesc: makeDefaultPatrolTaskDescription(),
    patrolWaypoints: ['waypoint_1', 'waypoint_2', 'waypoint_3'],
    onChange: () => {},
    onValidate: () => {},
  },
};
