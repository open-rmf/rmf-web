import { Meta, Story } from '@storybook/react';
import type { LiftState } from 'api-client';
import React from 'react';
import { makeLiftState } from '../lifts/test-utils.spec';
import { LiftMarker, LiftMarkerProps } from './lift-marker';

export default {
  title: 'Map/Lift Markers',
  component: LiftMarker,
} as Meta;

function makeStory(
  width: number,
  height: number,
  yaw: number,
  liftState?: LiftState,
  variant?: LiftMarkerProps['variant'],
): Story {
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

export const Basic = makeStory(1, 1, 0, makeLiftState());

export const UnknownState = makeStory(1, 1, 0, undefined, 'unknown');

export const Rotated = makeStory(1, 1, 45, makeLiftState());

export const LongLongLift = makeStory(2, 1, 0, makeLiftState());
