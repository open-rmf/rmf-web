import React from 'react';
import { Story, Meta } from '@storybook/react';
import { LogManagement } from '../lib';

export default {
  title: 'Log Management UI',
  component: LogManagement,
  argTypes: {},
} as Meta;

const getLabels = async () => {
  return [
    { label: 'Web Server', value: 'web-server' },
    { label: 'RMF core', value: 'rmf-core' },
  ];
};

const getLogs = async () => {
  const rows = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      message: 'Test' + i,
      level: 'WARN',
      timestamp: 'Mon Jan  1 00:00:02 UTC 2001',
    });
  }
  return rows;
};

export const LogManagementExample: Story = (args) => (
  <LogManagement getLogs={getLogs} getLabels={getLabels} {...args} />
);
