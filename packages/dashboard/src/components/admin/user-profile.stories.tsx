import { Meta, StoryFn } from '@storybook/react';
import { User } from 'api-client';
import React from 'react';
import { UserProfileCard, UserProfileCardProps } from '../../components/admin/user-profile';

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

export const Default: StoryFn<UserProfileCardProps> = (args) => {
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
};

Default.storyName = 'User Profile Card';
