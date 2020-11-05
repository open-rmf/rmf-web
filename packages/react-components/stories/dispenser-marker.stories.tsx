import { Meta, Story } from '@storybook/react';
import React from 'react';
import { DispenserMarker } from '../lib';

export default {
  title: 'Dispenser Marker',
  component: DispenserMarker,
  parameters: { actions: { argTypesRegex: '^on.*' } },
} as Meta;

export const Basic: Story = (args) => (
  <svg viewBox="-1 -1 2 2" width={400} height={400}>
    <DispenserMarker guid="test" {...args} />
  </svg>
);

export const Image: Story = (args) => (
  <svg viewBox="-1 -1 2 2" width={400} height={400}>
    <DispenserMarker guid="test" iconPath="/resources/ros-health.png" {...args} />
  </svg>
);
