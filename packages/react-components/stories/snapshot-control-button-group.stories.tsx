import { Story, Meta } from '@storybook/react';
import React from 'react';
import { SnapshotControlButtonGroup } from '../lib';

export default {
  title: 'Snapshot Control Button Group',
  component: SnapshotControlButtonGroup,
  argTypes: {
    fullWidth: { control: 'boolean' },
  },
} as Meta;

export const SimpleSnapshotControlButtonGroup: Story = (args) => (
  <SnapshotControlButtonGroup {...args} />
);
