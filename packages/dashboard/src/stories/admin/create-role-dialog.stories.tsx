import { Meta, Story } from '@storybook/react';
import React from 'react';
import { CreateRoleDialog, CreateRoleDialogProps } from '../../components/admin/create-role-dialog';

export default {
  title: 'Admin/Create Role Dialog',
  component: CreateRoleDialog,
} as Meta;

export const Default: Story<CreateRoleDialogProps> = (args) => {
  return (
    <CreateRoleDialog
      {...args}
      open={true}
      createRole={() => new Promise((res) => setTimeout(res, 100))}
    />
  );
};

Default.storyName = 'Create Role Dialog';
