import { Meta, StoryFn } from '@storybook/react';
import { RoleListCard, RoleListCardProps } from '../../components/admin/role-list-card';
import { RmfAction } from '../../components/permissions';

export default {
  title: 'Admin/Role List Card',
  component: RoleListCard,
} satisfies Meta;

export const Default: StoryFn<RoleListCardProps> = (args) => {
  return (
    <RoleListCard
      {...args}
      getRoles={async () => {
        await new Promise((res) => setTimeout(res, 100));
        return ['role4', 'role2', 'role3', 'role1'];
      }}
      getPermissions={async () => {
        await new Promise((res) => setTimeout(res, 100));
        return [
          { action: RmfAction.TaskCancel, authz_grp: 'group1' },
          { action: RmfAction.TaskRead, authz_grp: 'group1' },
        ];
      }}
      createRole={() => new Promise((res) => setTimeout(res, 100))}
      deleteRole={() => new Promise((res) => setTimeout(res, 100))}
    />
  );
};

Default.storyName = 'Role List Card';
