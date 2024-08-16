import { Meta, StoryObj } from '@storybook/react';

import { AdminDrawer } from './drawer';

export default {
  title: 'Admin/Drawer',
  component: AdminDrawer,
  argTypes: {
    active: {
      defaultValue: 'Users',
    },
  },
} satisfies Meta;

type Story = StoryObj<typeof AdminDrawer>;

export const Default: Story = {
  storyName: 'Drawer',
};
