import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { LiftMarker } from '../lib';
import { makeLift, makeLiftState } from '../tests/lifts/test-utils';

export default {
  title: 'Lift Markers',
  component: LiftMarker,
} as Meta;

function makeStory(
  lift: RomiCore.Lift,
  liftState?: RomiCore.LiftState,
  isInCurrentFloor = true,
): Story {
  return (args) => (
    <svg viewBox="-2 -2 4 4" width={400} height={400}>
      <LiftMarker lift={lift} liftState={liftState} isInCurrentFloor={isInCurrentFloor} {...args} />
    </svg>
  );
}

export const Basic = makeStory(makeLift(), makeLiftState());

export const UnknownState = makeStory(makeLift());

export const Rotated = makeStory(
  makeLift({
    ref_yaw: Math.PI / 4,
    doors: [
      {
        name: 'door',
        door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE,
        motion_direction: 1,
        motion_range: Math.PI / 2,
        v1_x: -1.414,
        v1_y: 0,
        v2_x: 0,
        v2_y: -1.414,
      },
    ],
  }),
  makeLiftState(),
);

// FIXME: Broken
export const LongLongLift = makeStory(
  makeLift({
    width: 2,
    depth: 1,
    doors: [
      {
        name: 'door',
        door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE,
        motion_direction: 1,
        motion_range: Math.PI / 2,
        v1_x: -1,
        v1_y: -1,
        v2_x: 2,
        v2_y: -1,
      },
    ],
  }),
  makeLiftState(),
);
