import { Meta, Story } from '@storybook/react';
import React from 'react';
import { CreateUserDialog, CreateUserDialogProps } from '../../components/admin/create-user-dialog';

export default {
  title: 'Admin/Create User Dialog',
  component: CreateUserDialog,
} as Meta;

export const Default: Story<CreateUserDialogProps> = (args) => {
  return (
    <CreateUserDialog
      {...args}
      open={true}
      createUser={() => new Promise((res) => setTimeout(res, 100))}
    />
  );
};

Default.storyName = 'Create User Dialog';
