import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { WaypointMarker } from './waypoint-marker';

export default {
  title: 'Map/Waypoint Marker',
  component: WaypointMarker,
  parameters: { actions: { argTypesRegex: '^on.*' } },
} satisfies Meta;

export const Default: StoryFn = (args) => {
  return (
    <svg viewBox="-2 -2 4 4" width={400} height={400}>
      <WaypointMarker cx={0} cy={0} size={1} {...args} />
    </svg>
  );
};

Default.storyName = 'Waypoint Marker';
