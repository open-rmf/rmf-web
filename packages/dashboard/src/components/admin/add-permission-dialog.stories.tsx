import { Meta, StoryObj } from '@storybook/react';

import { AddPermissionDialog } from './add-permission-dialog';

export default {
  title: 'Admin/Add Permission Dialog',
  component: AddPermissionDialog,
} satisfies Meta;

type Story = StoryObj<typeof AddPermissionDialog>;

export const Default: Story = {
  storyName: 'Add Permission Dialog',
  render: (args) => (
    <AddPermissionDialog
      {...args}
      open={true}
      setOpen={() => {}}
      savePermission={() => new Promise((res) => setTimeout(res, 100))}
    />
  ),
};
