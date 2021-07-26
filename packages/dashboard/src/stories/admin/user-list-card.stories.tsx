import { Meta, Story } from '@storybook/react';
import React from 'react';
import { MemoryRouter } from 'react-router';
import { UserListCard, UserListCardProps } from '../../components/admin/user-list-card';

export default {
  title: 'Admin/User List Card',
  component: UserListCard,
} as Meta;

const users: string[] = [];
for (let i = 0; i < 100; i++) {
  users.push(`user${i + 1}`);
}

async function searchUsers(search: string, limit: number, offset: number) {
  await new Promise((res) => setTimeout(res, 100));
  return users.filter((u) => u.startsWith(search)).slice(offset, offset + limit);
}

export const Default: Story<UserListCardProps> = (args) => {
  return (
    <MemoryRouter>
      <UserListCard
        {...args}
        searchUsers={searchUsers}
        deleteUser={() => new Promise((res) => setTimeout(res, 100))}
        createUser={() => new Promise((res) => setTimeout(res, 100))}
      />
    </MemoryRouter>
  );
};

Default.storyName = 'User List Card';
