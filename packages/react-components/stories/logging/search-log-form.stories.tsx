import { Story, Meta } from '@storybook/react';
import React from 'react';
import { SearchLogForm } from '../../lib';

export default {
  title: 'Search log Form',
  component: SearchLogForm,
  argTypes: {},
} as Meta;

const logLabel = [
  { label: 'Web Server', value: 'web-server' },
  { label: 'RMF core', value: 'rmf-core' },
];
export const SimpleSearchLogForm: Story = (args) => (
  <SearchLogForm logLabelValues={logLabel} {...args} />
);
