import { Meta, Story } from '@storybook/react';
import React from 'react';
import { RoleListCard, RoleListCardProps } from '../../components/admin/role-list';

export default {
  title: 'Admin/Role List Card',
  component: RoleListCard,
  argTypes: {
    roles: {
      defaultValue: ['role1', 'role2', 'role3', 'role4'],
    },
  },
} as Meta;

export const Default: Story<RoleListCardProps> = (args) => {
  return <RoleListCard {...args} />;
};

Default.storyName = 'Role List Card';
