import { Meta, Story } from '@storybook/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { makeDoorState } from '../doors/test-utils.spec';
import { DoorMarker } from './door-marker';

export default {
  title: 'Map/Door Markers',
  component: DoorMarker,
} as Meta;

function makeStory(doorType: number, doorState?: RmfModels.DoorState): Story {
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

export const SingleSwing = makeStory(RmfModels.Door.DOOR_TYPE_SINGLE_SWING);
export const SingleSliding = makeStory(RmfModels.Door.DOOR_TYPE_SINGLE_SLIDING);
export const SingleTelescope = makeStory(RmfModels.Door.DOOR_TYPE_SINGLE_TELESCOPE);
export const DoubleSwing = makeStory(RmfModels.Door.DOOR_TYPE_DOUBLE_SWING);
export const DoubleSliding = makeStory(RmfModels.Door.DOOR_TYPE_DOUBLE_SLIDING);
export const DoubleTelescope = makeStory(RmfModels.Door.DOOR_TYPE_DOUBLE_TELESCOPE);

export const SingleSwingOpened = makeStory(
  RmfModels.Door.DOOR_TYPE_SINGLE_SWING,
  makeDoorState({ current_mode: { value: RmfModels.DoorMode.MODE_OPEN } }),
);
SingleSwingOpened.storyName = 'Single Swing (Opened)';
export const SingleSwingMoving = makeStory(
  RmfModels.Door.DOOR_TYPE_SINGLE_SWING,
  makeDoorState({ current_mode: { value: RmfModels.DoorMode.MODE_MOVING } }),
);
SingleSwingMoving.storyName = 'Single Swing (Moving)';
export const SingleSwingClosed = makeStory(
  RmfModels.Door.DOOR_TYPE_SINGLE_SWING,
  makeDoorState({ current_mode: { value: RmfModels.DoorMode.MODE_CLOSED } }),
);
SingleSwingClosed.storyName = 'Single Swing (Closed)';
