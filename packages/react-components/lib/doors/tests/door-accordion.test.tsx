import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { fireEvent, render } from '@testing-library/react';
import React from 'react';
import { DoorAccordion } from '..';

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

test('triggers onDoorOpen callback when button is clicked', () => {
  const handleDoorOpen = jest.fn();
  const root = render(<DoorAccordion door={makeDoor()} doOpenDoor={handleDoorOpen} />);
  fireEvent.click(root.getByTestId('open'));
  expect(handleDoorOpen).toBeCalled();
});

test('triggers onDoorClose callback when button is clicked', () => {
  const handleDoorClose = jest.fn();
  const root = render(<DoorAccordion door={makeDoor()} doCloseDoor={handleDoorClose} />);
  fireEvent.click(root.getByTestId('close'));
  expect(handleDoorClose).toBeCalled();
});
