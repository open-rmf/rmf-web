import { Meta, Story } from '@storybook/react';
import React from 'react';
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
} as Meta;

export const LoginCard: Story<LoginCardProps> = (args) => (
  <LoginCard_ {...args} logo="/resources/ros-health.png" />
);

export const LoginPage: Story<LoginCardProps> = (args) => (
  <LoginPage_ {...args} logo="/resources/ros-health.png" />
);
