import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { WorkcellMarker } from './workcell-marker';

export default {
  title: 'Map/Workcell Marker',
  component: WorkcellMarker,
  parameters: { actions: { argTypesRegex: '^on.*' } },
} satisfies Meta;

export const Basic: StoryFn = (args) => (
  <svg viewBox="-1 -1 2 2" width={400} height={400}>
    <WorkcellMarker cx={0} cy={0} size={1} {...args} />
  </svg>
);

export const Image: StoryFn = (args) => (
  <svg viewBox="-1 -1 2 2" width={400} height={400}>
    <WorkcellMarker cx={0} cy={0} size={1} iconPath="/assets/ros-health.png" {...args} />
  </svg>
);
