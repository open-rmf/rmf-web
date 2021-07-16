import { Meta, Story } from '@storybook/react';
import React from 'react';
import { AddRoleDialog, AddRoleDialogProps } from '../../components/admin/add-role-dialog';

export default {
  title: 'Admin/Add Role Dialog',
  component: AddRoleDialog,
} as Meta;

export const Default: Story<AddRoleDialogProps> = (args) => {
  return (
    <AddRoleDialog
      {...args}
      open={true}
      createRole={() => new Promise((res) => setTimeout(res, 500))}
    />
  );
};

Default.storyName = 'Add Role Dialog';
