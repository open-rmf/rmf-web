import { Meta, StoryObj } from '@storybook/react';

import { CreateUserDialog } from './create-user-dialog';

export default {
  title: 'Admin/Create User Dialog',
  component: CreateUserDialog,
} satisfies Meta;

type Story = StoryObj<typeof CreateUserDialog>;

export const Default: Story = {
  storyName: 'Create User Dialog',
  render: (args) => (
    <CreateUserDialog
      {...args}
      open={true}
      createUser={() => new Promise((res) => setTimeout(res, 100))}
    />
  ),
};

Default.storyName = 'Create User Dialog';
