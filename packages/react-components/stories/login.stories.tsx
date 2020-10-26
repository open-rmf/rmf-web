import { Meta, Story } from '@storybook/react';
import React from 'react';
import { LoginCard as LoginCard_, LoginPage as LoginPage_ } from '../lib';

export default {
  title: 'Login',
  component: LoginCard_,
} as Meta;

export const LoginCard: Story = (args) => <LoginCard_ logo="/resources/ros-health.png" {...args} />;
export const LoginPage: Story = (args) => <LoginPage_ logo="/resources/ros-health.png" {...args} />;
