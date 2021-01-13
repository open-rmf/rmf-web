import { Story, Meta } from '@storybook/react';
import React from 'react';
import { SearchLogForm } from '../lib';

export default {
  title: 'Search log Form',
  component: SearchLogForm,
  argTypes: {
    fullWidth: { control: 'boolean' },
  },
} as Meta;

export const SimpleSearchLogForm: Story = (args) => <SearchLogForm {...args} />;
