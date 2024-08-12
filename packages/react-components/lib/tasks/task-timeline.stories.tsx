import { Meta, StoryObj } from '@storybook/react';

import { TaskTimeline } from './task-timeline';
import { makeTaskState } from './test-data.spec';

export default {
  title: 'Tasks/Timeline',
  component: TaskTimeline,
} satisfies Meta;

type Story = StoryObj<typeof TaskTimeline>;

export const Timeline: Story = {
  args: {
    taskState: makeTaskState('task'),
  },
};
