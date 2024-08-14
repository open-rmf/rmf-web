import { Meta, StoryObj } from '@storybook/react';

import { LoginCard } from './login-card';

export default {
  title: 'Login',
  component: LoginCard,
  argTypes: {
    title: {
      defaultValue: 'Title',
    },
    logo: {
      control: {
        disable: true,
      },
    },
  },
} satisfies Meta;

type Story = StoryObj<typeof LoginCard>;

export const Default: Story = {
  storyName: 'Login Card',
  render: (args) => <LoginCard {...args} logo="/resources/ros-health.png" />,
};
