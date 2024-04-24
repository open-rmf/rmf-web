import { Meta, StoryFn } from '@storybook/react';
import { CreateRoleDialog, CreateRoleDialogProps } from '../../components/admin/create-role-dialog';

export default {
  title: 'Admin/Create Role Dialog',
  component: CreateRoleDialog,
} satisfies Meta;

export const Default: StoryFn<CreateRoleDialogProps> = (args) => {
  return (
    <CreateRoleDialog
      {...args}
      open={true}
      createRole={() => new Promise((res) => setTimeout(res, 100))}
    />
  );
};

Default.storyName = 'Create Role Dialog';
