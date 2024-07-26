import { Meta, StoryFn } from '@storybook/react';
import { Permission } from 'api-client';
import { PermissionsCard, PermissionsCardProps } from './permissions-card';

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

export const Default: StoryFn<PermissionsCardProps> = (args) => {
  return (
    <PermissionsCard {...args} savePermission={() => new Promise((res) => setTimeout(res, 100))} />
  );
};

Default.storyName = 'Permissions Card';
