import { Meta, StoryObj } from '@storybook/react';

import { RmfAction } from '../../services/permissions';
import { RoleListCard } from './role-list-card';

export default {
  title: 'Admin/Role List Card',
  component: RoleListCard,
} satisfies Meta;

type Story = StoryObj<typeof RoleListCard>;

export const Default: Story = {
  storyName: 'Role List Card',
  render: (args) => (
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
  ),
};
