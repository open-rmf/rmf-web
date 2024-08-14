import { Meta, StoryObj } from '@storybook/react';
import { Permission } from 'api-client';

import { PermissionsCard } from './permissions-card';

export default {
  title: 'Admin/Permissions Card',
  component: PermissionsCard,
  argTypes: {
    permissions: {
      defaultValue: [
        { action: 'task_submit', authz_grp: 'group1' },
        { action: 'task_read', authz_grp: 'group1' },
      ] as Permission[],
    },
  },
} satisfies Meta;

type Story = StoryObj<typeof PermissionsCard>;

export const Default: Story = {
  storyName: 'Permissions Card',
  render: (args) => (
    <PermissionsCard {...args} savePermission={() => new Promise((res) => setTimeout(res, 100))} />
  ),
};
