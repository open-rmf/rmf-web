import { Meta, StoryObj } from '@storybook/react';

import { LinearProgressBar } from './linear-progress-bar';

export default {
  title: 'Robots/Linear Progress Bar',
  component: LinearProgressBar,
} satisfies Meta;

type Story = StoryObj<typeof LinearProgressBar>;

export const Default: Story = {
  args: {
    value: 100,
  },
};
