import { Meta, StoryFn } from '@storybook/react';
import { AdminDrawer } from '../../components/admin/drawer';

export default {
  title: 'Admin/Drawer',
  component: AdminDrawer,
  argTypes: {
    active: {
      defaultValue: 'Users',
    },
  },
} satisfies Meta;

export const Default: StoryFn<{}> = (args) => {
  return <AdminDrawer {...args} />;
};

Default.storyName = 'Drawer';
