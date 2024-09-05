import { Meta, StoryObj } from '@storybook/react';

import { LoginCardProps } from '../components/login-card';
import { LoginPage } from './login-page';

export default {
  title: 'Login',
  component: LoginPage,
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

type Story = StoryObj<typeof LoginPage>;

export const Default: Story = {
  storyName: 'LoginPage',
  render: (args: LoginCardProps) => <LoginPage {...args} logo="/resources/ros-health.png" />,
};
