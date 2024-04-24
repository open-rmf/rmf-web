import { Meta, StoryFn } from '@storybook/react';
import type { LiftState } from 'api-client';
import React from 'react';
import { makeLiftState } from '../lifts/test-utils.spec';
import { LiftMarker, LiftMarkerProps } from './lift-marker';

export default {
  title: 'Map/Lift Markers',
  component: LiftMarker,
} satisfies Meta;

function makeStory(
  width: number,
  height: number,
  yaw: number,
  liftState?: LiftState,
  variant?: LiftMarkerProps['variant'],
): StoryFn {
  return (args) => (
    <svg viewBox="-2 -2 4 4" width={400} height={400}>
      <LiftMarker
        cx={0}
        cy={0}
        width={width}
        height={height}
        yaw={yaw}
        liftState={liftState}
        variant={variant}
        {...args}
      />
    </svg>
  );
}

export const Basic: StoryFn = makeStory(1, 1, 0, makeLiftState());

export const UnknownState: StoryFn = makeStory(1, 1, 0, undefined, 'unknown');

export const Rotated: StoryFn = makeStory(1, 1, 45, makeLiftState());

export const LongLongLift: StoryFn = makeStory(2, 1, 0, makeLiftState());
