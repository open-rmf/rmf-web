import { Meta, Story } from '@storybook/react';
import type { DoorState } from 'api-client';
import React from 'react';
import { Door as RmfDoor, DoorMode as RmfDoorMode } from 'rmf-models';
import { makeDoorState } from '../doors/test-utils.spec';
import { DoorMarker } from './door-marker';

export default {
  title: 'Map/Door Markers',
  component: DoorMarker,
} as Meta;

function makeStory(doorType: number, doorState?: DoorState): Story {
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

export const SingleSwing = makeStory(RmfDoor.DOOR_TYPE_SINGLE_SWING);
export const SingleSliding = makeStory(RmfDoor.DOOR_TYPE_SINGLE_SLIDING);
export const SingleTelescope = makeStory(RmfDoor.DOOR_TYPE_SINGLE_TELESCOPE);
export const DoubleSwing = makeStory(RmfDoor.DOOR_TYPE_DOUBLE_SWING);
export const DoubleSliding = makeStory(RmfDoor.DOOR_TYPE_DOUBLE_SLIDING);
export const DoubleTelescope = makeStory(RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE);

export const SingleSwingOpened = makeStory(
  RmfDoor.DOOR_TYPE_SINGLE_SWING,
  makeDoorState({ current_mode: { value: RmfDoorMode.MODE_OPEN } }),
);
SingleSwingOpened.storyName = 'Single Swing (Opened)';
export const SingleSwingMoving = makeStory(
  RmfDoor.DOOR_TYPE_SINGLE_SWING,
  makeDoorState({ current_mode: { value: RmfDoorMode.MODE_MOVING } }),
);
SingleSwingMoving.storyName = 'Single Swing (Moving)';
export const SingleSwingClosed = makeStory(
  RmfDoor.DOOR_TYPE_SINGLE_SWING,
  makeDoorState({ current_mode: { value: RmfDoorMode.MODE_CLOSED } }),
);
SingleSwingClosed.storyName = 'Single Swing (Closed)';
