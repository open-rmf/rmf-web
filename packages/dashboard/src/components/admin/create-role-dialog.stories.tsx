import { Meta, StoryObj } from '@storybook/react';

import { CreateRoleDialog } from './create-role-dialog';

export default {
  title: 'Admin/Create Role Dialog',
  component: CreateRoleDialog,
} satisfies Meta;

type Story = StoryObj<typeof CreateRoleDialog>;

export const Default: Story = {
  storyName: 'Create Role Dialog',
  render: (args) => (
    <CreateRoleDialog
      {...args}
      open={true}
      createRole={() => new Promise((res) => setTimeout(res, 100))}
    />
  ),
};
