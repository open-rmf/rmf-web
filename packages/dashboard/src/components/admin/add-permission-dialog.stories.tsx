import { Meta, StoryFn } from '@storybook/react';
import {
  AddPermissionDialog,
  AddPermissionDialogProps,
} from '../../components/admin/add-permission-dialog';

export default {
  title: 'Admin/Add Permission Dialog',
  component: AddPermissionDialog,
} satisfies Meta;

export const Default: StoryFn<AddPermissionDialogProps> = (args) => {
  return (
    <AddPermissionDialog
      {...args}
      open={true}
      setOpen={() => {}}
      savePermission={() => new Promise((res) => setTimeout(res, 100))}
    />
  );
};

Default.storyName = 'Add Permission Dialog';
