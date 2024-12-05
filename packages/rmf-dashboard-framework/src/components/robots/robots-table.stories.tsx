import { Meta, StoryObj } from '@storybook/react';

import { RobotsTable } from './robots-table';

export default {
  title: 'Robots/RobotsTable',
  component: RobotsTable,
} satisfies Meta;

type Story = StoryObj<typeof RobotsTable>;

export const Default: Story = {
  args: {},
};
