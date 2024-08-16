import { Meta, StoryObj } from '@storybook/react';
import { User } from 'api-client';
import React from 'react';

import { UserProfileCard } from './user-profile';

export default {
  title: 'Admin/User Profile Card',
  component: UserProfileCard,
  argTypes: {
    profile: {
      control: {
        disable: true,
      },
    },
  },
} satisfies Meta;

type Story = StoryObj<typeof UserProfileCard>;

export const Default: Story = {
  storyName: 'User Profile Card',
  render: (args) => {
    const [user, setUser] = React.useState<User>({
      username: 'example',
      is_admin: false,
      roles: [],
    });
    return (
      <UserProfileCard
        {...args}
        user={user}
        makeAdmin={async (admin) => {
          await new Promise((res) => setTimeout(res, 100));
          setUser((prev) => ({ ...prev, is_admin: admin }));
        }}
      ></UserProfileCard>
    );
  },
};
