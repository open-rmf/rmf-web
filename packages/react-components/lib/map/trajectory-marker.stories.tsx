import { Meta, StoryFn } from '@storybook/react';
import React from 'react';
import { makeTrajectory } from './test-utils.spec';
import { TrajectoryMarker, TrajectoryMarkerProps } from './trajectory-marker';

export default {
  title: 'Map/Trajectory Markers',
  component: TrajectoryMarker,
  argTypes: {
    color: {
      control: {
        type: 'color',
      },
    },
    trajectory: {
      table: {
        disable: true,
      },
    },
  },
} satisfies Meta;

const trajectory = makeTrajectory();

export const Basic: StoryFn<Omit<TrajectoryMarkerProps, 'trajectory'>> = (args) => {
  return (
    <svg viewBox="-2 -2 16 16" width={400} height={400}>
      <TrajectoryMarker trajectory={trajectory} {...args} />
    </svg>
  );
};
Basic.args = {
  color: 'green',
  loopAnimation: true,
};
