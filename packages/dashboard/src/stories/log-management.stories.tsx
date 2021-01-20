import React from 'react';
import { Story, Meta } from '@storybook/react';
import { LogManagementApp } from '../components/log-management';

export default {
  title: 'Log Management UI',
  component: LogManagementApp,
  argTypes: {},
} as Meta;

export const LogManagement: Story = (args) => <LogManagementApp {...args} />;
