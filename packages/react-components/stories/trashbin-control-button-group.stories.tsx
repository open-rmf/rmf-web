import { Story, Meta } from '@storybook/react';
import React from 'react';
import { TrashBinControlButtonGroup } from '../lib';

export default {
  title: 'Trashbin Control Button Group',
  component: TrashBinControlButtonGroup,
  argTypes: {
    fullWidth: { control: 'boolean' },
  },
} as Meta;

export const SimpleTrashBinControlButtonGroup: Story = (args) => (
  <TrashBinControlButtonGroup {...args} />
);
