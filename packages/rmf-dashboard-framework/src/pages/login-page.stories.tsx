import { Meta, StoryObj } from '@storybook/react';

import { LoginCardProps } from '../components/login-card';
import { LoginPage } from './login-page';

export default {
  title: 'Login Page',
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
  name: 'LoginPage',
  render: (args: LoginCardProps) => <LoginPage {...args} logo="/resources/defaultLogo.png" />,
};
