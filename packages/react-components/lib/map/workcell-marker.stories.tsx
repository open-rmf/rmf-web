import { Meta, Story } from '@storybook/react';
import React from 'react';
import { WorkcellMarker } from './workcell-marker';

export default {
  title: 'Map/Workcell Marker',
  component: WorkcellMarker,
  parameters: { actions: { argTypesRegex: '^on.*' } },
} as Meta;

export const Basic: Story = (args) => (
  <svg viewBox="-1 -1 2 2" width={400} height={400}>
    <WorkcellMarker cx={0} cy={0} size={1} {...args} />
  </svg>
);

export const Image: Story = (args) => (
  <svg viewBox="-1 -1 2 2" width={400} height={400}>
    <WorkcellMarker cx={0} cy={0} size={1} iconPath="/assets/ros-health.png" {...args} />
  </svg>
);
