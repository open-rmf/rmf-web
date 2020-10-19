import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { fireEvent, render } from '@testing-library/react';
import React from 'react';
import { DoorItem } from '..';

function makeDoor(door?: Partial<RomiCore.Door>): RomiCore.Door {
  return {
    name: 'test',
    door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SWING,
    motion_direction: 1,
    motion_range: Math.PI / 2,
    v1_x: 0,
    v1_y: 0,
    v2_x: 1,
    v2_y: 1,
    ...door,
  };
}

function makeDoorComponent(
  door?: Partial<RomiCore.Door>,
  state?: Partial<RomiCore.DoorState>,
): JSX.Element {
  const state_: RomiCore.DoorState | undefined =
    state === undefined
      ? undefined
      : {
          door_name: 'test',
          door_time: { sec: 0, nanosec: 0 },
          current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
          ...state,
        };

  return <DoorItem door={makeDoor(door)} doorState={state_} />;
}

[
  {
    Component: makeDoorComponent({ door_type: RomiCore.Door.DOOR_TYPE_UNDEFINED }),
    type: 'undefined',
  },
  {
    Component: makeDoorComponent({ door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING }),
    type: 'single sliding',
  },
  {
    Component: makeDoorComponent({ door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING }),
    type: 'double sliding',
  },
  {
    Component: makeDoorComponent({ door_type: RomiCore.Door.DOOR_TYPE_SINGLE_TELESCOPE }),
    type: 'single telescope',
  },
  {
    Component: makeDoorComponent({ door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE }),
    type: 'double telescope',
  },
  {
    Component: makeDoorComponent({ door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SWING }),
    type: 'single swing',
  },
  {
    Component: makeDoorComponent({ door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SWING }),
    type: 'double swing',
  },
  {
    Component: makeDoorComponent({
      door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
      motion_direction: -1,
    }),
    type: 'single sliding; anti clockwise',
  },
  {
    Component: makeDoorComponent({
      door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
      motion_direction: 2,
    }),
    type: 'single sliding; unknown motion',
  },
  {
    Component: makeDoorComponent(
      { door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING },
      { current_mode: { value: RomiCore.DoorMode.MODE_OPEN } },
    ),
    type: 'single sliding',
    state: 'open',
  },
  {
    Component: makeDoorComponent(
      { door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING },
      { current_mode: { value: RomiCore.DoorMode.MODE_CLOSED } },
    ),
    type: 'single sliding',
    state: 'closed',
  },
  {
    Component: makeDoorComponent(
      { door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING },
      { current_mode: { value: RomiCore.DoorMode.MODE_MOVING } },
    ),
    type: 'single sliding',
    state: 'moving',
  },
  {
    Component: makeDoorComponent(
      { door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING },
      { current_mode: { value: -1 } },
    ),
    type: 'single sliding',
    state: 'unknown',
  },
].map((params) => {
  test(`renders [type='${params.type}', state='${params.state}'] correctly`, () => {
    const root = render(params.Component);
    expect(root.container).toMatchSnapshot();
  });
});

test('triggers onDoorOpen callback when button is clicked', () => {
  const handleDoorOpen = jest.fn();
  const root = render(<DoorItem door={makeDoor()} onDoorOpen={handleDoorOpen} />);
  fireEvent.click(root.getByTestId('open'));
  expect(handleDoorOpen).toBeCalled();
});

test('triggers onDoorClose callback when button is clicked', () => {
  const handleDoorClose = jest.fn();
  const root = render(<DoorItem door={makeDoor()} onDoorClose={handleDoorClose} />);
  fireEvent.click(root.getByTestId('close'));
  expect(handleDoorClose).toBeCalled();
});
