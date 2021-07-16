import { Meta, Story } from '@storybook/react';
import React from 'react';
import { AddUserDialog, AddUserDialogProps } from '../../components/admin/add-user-dialog';

export default {
  title: 'Admin/Add User Dialog',
  component: AddUserDialog,
} as Meta;

export const Default: Story<AddUserDialogProps> = (args) => {
  return (
    <AddUserDialog
      {...args}
      open={true}
      createUser={() => new Promise((res) => setTimeout(res, 500))}
    />
  );
};

Default.storyName = 'Add User Dialog';
