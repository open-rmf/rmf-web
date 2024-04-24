import { Meta, StoryFn } from '@storybook/react';
import type { DoorState } from 'api-client';
import React from 'react';
import { Door as RmfDoor, DoorMode as RmfDoorMode } from 'rmf-models';
import { makeDoorState } from '../doors/test-utils.spec';
import { DoorMarker } from './door-marker';

export default {
  title: 'Map/Door Markers',
  component: DoorMarker,
} satisfies Meta;

function makeStory(doorType: number, doorState?: DoorState): StoryFn {
  return (args) => (
    <svg viewBox="-2 -2 4 4" width={400} height={400}>
      <DoorMarker
        x1={-1}
        y1={0}
        x2={1}
        y2={0}
        doorType={doorType}
        doorMode={doorState?.current_mode.value || undefined}
        {...args}
      />
    </svg>
  );
}

export const SingleSwing: StoryFn = makeStory(RmfDoor.DOOR_TYPE_SINGLE_SWING);
export const SingleSliding: StoryFn = makeStory(RmfDoor.DOOR_TYPE_SINGLE_SLIDING);
export const SingleTelescope: StoryFn = makeStory(RmfDoor.DOOR_TYPE_SINGLE_TELESCOPE);
export const DoubleSwing: StoryFn = makeStory(RmfDoor.DOOR_TYPE_DOUBLE_SWING);
export const DoubleSliding: StoryFn = makeStory(RmfDoor.DOOR_TYPE_DOUBLE_SLIDING);
export const DoubleTelescope: StoryFn = makeStory(RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE);

export const SingleSwingOpened: StoryFn = makeStory(
  RmfDoor.DOOR_TYPE_SINGLE_SWING,
  makeDoorState({ current_mode: { value: RmfDoorMode.MODE_OPEN } }),
);
SingleSwingOpened.storyName = 'Single Swing (Opened)';
export const SingleSwingMoving: StoryFn = makeStory(
  RmfDoor.DOOR_TYPE_SINGLE_SWING,
  makeDoorState({ current_mode: { value: RmfDoorMode.MODE_MOVING } }),
);
SingleSwingMoving.storyName = 'Single Swing (Moving)';
export const SingleSwingClosed: StoryFn = makeStory(
  RmfDoor.DOOR_TYPE_SINGLE_SWING,
  makeDoorState({ current_mode: { value: RmfDoorMode.MODE_CLOSED } }),
);
SingleSwingClosed.storyName = 'Single Swing (Closed)';
