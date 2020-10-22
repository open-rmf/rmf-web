import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { DoorMarker } from '../lib';

export default {
  title: 'Door Markers',
  component: DoorMarker,
  argTypes: { onClick: { action: 'clicked' } },
} as Meta;

const baseDoor: RomiCore.Door = {
  name: 'test',
  v1_x: 0,
  v1_y: 0,
  v2_x: 1,
  v2_y: 1,
  door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SWING,
  motion_range: -1.571,
  motion_direction: 1,
};

const singleSwingDoor: RomiCore.Door = {
  ...baseDoor,
  door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SWING,
};

const singleSlidingDoor: RomiCore.Door = {
  ...baseDoor,
  door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
};

const doubleSwingDoor: RomiCore.Door = {
  ...baseDoor,
  door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SWING,
};

const doubleSlidingDoor: RomiCore.Door = {
  ...baseDoor,
  door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING,
};

const baseState: RomiCore.DoorState = {
  door_name: 'test',
  current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
  door_time: { sec: 0, nanosec: 0 },
};

const openState: RomiCore.DoorState = {
  ...baseState,
  current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
};

const movingState: RomiCore.DoorState = {
  ...baseState,
  current_mode: { value: RomiCore.DoorMode.MODE_MOVING },
};

const closedState: RomiCore.DoorState = {
  ...baseState,
  current_mode: { value: RomiCore.DoorMode.MODE_CLOSED },
};

function makeStory(door: RomiCore.Door, doorState?: RomiCore.DoorState): Story {
  return (args) => (
    <svg viewBox="-2 -2 4 4" width={400} height={400}>
      <DoorMarker door={door} doorState={doorState} {...args} />
    </svg>
  );
}

export const SingleSwing = makeStory(singleSwingDoor);
export const SingleSliding = makeStory(singleSlidingDoor);
export const DoubleSwing = makeStory(doubleSwingDoor);
export const DoubleSliding = makeStory(doubleSlidingDoor);

export const Opened = makeStory(baseDoor, openState);
export const Moving = makeStory(baseDoor, movingState);
export const Closed = makeStory(baseDoor, closedState);
