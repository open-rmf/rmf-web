import { Meta, StoryObj } from '@storybook/react';

import { ManageRolesCard } from './manage-roles-dialog';

export default {
  title: 'Admin/Manage Roles Card',
  component: ManageRolesCard,
} satisfies Meta;

const allRoles: string[] = [];
for (let i = 0; i < 5; i++) {
  allRoles.push(`role${i}`);
}

type Story = StoryObj<typeof ManageRolesCard>;

export const Default: Story = {
  storyName: 'Manage Roles Card',
  args: {
    assignedRoles: ['role1'],
  },
  render: (args) => (
    <ManageRolesCard
      getAllRoles={async () => {
        await new Promise((res) => setTimeout(res, 100));
        return allRoles;
      }}
      saveRoles={async () => {
        await new Promise((res) => setTimeout(res, 100));
      }}
      {...args}
    />
  ),
};
