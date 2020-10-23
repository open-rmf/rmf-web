import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { DoorMarker } from '../lib';
import { makeDoor, makeDoorState } from '../lib/doors/tests/test-utils';

export default {
  title: 'Door Markers',
  component: DoorMarker,
  argTypes: { onClick: { action: 'clicked' } },
} as Meta;

function makeStory(door: RomiCore.Door, doorState?: RomiCore.DoorState): Story {
  return (args) => (
    <svg viewBox="-2 -2 4 4" width={400} height={400}>
      <DoorMarker door={door} doorState={doorState} {...args} />
    </svg>
  );
}

export const SingleSwing = makeStory(makeDoor({ door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SWING }));
export const SingleSliding = makeStory(
  makeDoor({ door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING }),
);
export const SingleTelescope = makeStory(
  makeDoor({ door_type: RomiCore.Door.DOOR_TYPE_SINGLE_TELESCOPE }),
);
export const DoubleSwing = makeStory(makeDoor({ door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SWING }));
export const DoubleSliding = makeStory(
  makeDoor({ door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING }),
);
export const DoubleTelescope = makeStory(
  makeDoor({ door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE }),
);

export const SingleSwingOpened = makeStory(
  makeDoor(),
  makeDoorState({ current_mode: { value: RomiCore.DoorMode.MODE_OPEN } }),
);
SingleSwingOpened.storyName = 'Single Swing (Opened)';
export const SingleSwingMoving = makeStory(
  makeDoor(),
  makeDoorState({ current_mode: { value: RomiCore.DoorMode.MODE_MOVING } }),
);
SingleSwingMoving.storyName = 'Single Swing (Moving)';
export const SingleSwingClosed = makeStory(
  makeDoor(),
  makeDoorState({ current_mode: { value: RomiCore.DoorMode.MODE_CLOSED } }),
);
SingleSwingClosed.storyName = 'Single Swing (Closed)';
