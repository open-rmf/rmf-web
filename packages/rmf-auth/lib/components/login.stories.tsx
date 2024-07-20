import { Meta, StoryFn } from '@storybook/react';
import { LoginCard as LoginCard_, LoginPage as LoginPage_ } from '.';
import { LoginCardProps } from './login-card';

export default {
  title: 'Login',
  component: LoginCard_,
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

export const LoginCard: StoryFn<LoginCardProps> = (args) => (
  <LoginCard_ {...args} logo="/resources/ros-health.png" />
);

export const LoginPage: StoryFn<LoginCardProps> = (args) => (
  <LoginPage_ {...args} logo="/resources/ros-health.png" />
);
