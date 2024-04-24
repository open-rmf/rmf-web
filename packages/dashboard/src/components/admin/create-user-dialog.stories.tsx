import { Meta, StoryFn } from '@storybook/react';
import { CreateUserDialog, CreateUserDialogProps } from '../../components/admin/create-user-dialog';

export default {
  title: 'Admin/Create User Dialog',
  component: CreateUserDialog,
} satisfies Meta;

export const Default: StoryFn<CreateUserDialogProps> = (args) => {
  return (
    <CreateUserDialog
      {...args}
      open={true}
      createUser={() => new Promise((res) => setTimeout(res, 100))}
    />
  );
};

Default.storyName = 'Create User Dialog';
