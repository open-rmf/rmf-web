import { Meta, StoryFn } from '@storybook/react';
import { ManageRolesCard } from './manage-roles-dialog';
import { RoleListCardProps } from './role-list-card';

export default {
  title: 'Admin/Manage Roles Card',
} satisfies Meta;

const allRoles: string[] = [];
for (let i = 0; i < 5; i++) {
  allRoles.push(`role${i}`);
}

export const Default: StoryFn<RoleListCardProps> = (args) => {
  return (
    <ManageRolesCard
      assignedRoles={['role1']}
      getAllRoles={async () => {
        await new Promise((res) => setTimeout(res, 100));
        return allRoles;
      }}
      saveRoles={async () => {
        await new Promise((res) => setTimeout(res, 100));
      }}
      {...args}
    />
  );
};

Default.storyName = 'Manage Roles Card';
