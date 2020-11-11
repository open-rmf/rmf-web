import { Story, Meta } from '@storybook/react';
import React from 'react';
import { TreeButtonGroup } from '../lib';

export default {
  title: 'Tree Button Group',
  component: TreeButtonGroup,
  argTypes: {
    fullWidth: { control: 'boolean' },
  },
} as Meta;

export const SimpleTreeButtonGroup: Story = (args) => <TreeButtonGroup {...args} />;
