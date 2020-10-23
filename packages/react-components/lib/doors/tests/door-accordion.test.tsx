import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
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

test('triggers doOpenDoor callback when button is clicked', () => {
  const handleDoorOpen = jest.fn();
  const root = render(<DoorAccordion door={makeDoor()} onOpenClick={handleDoorOpen} />);
  userEvent.click(root.getByText('Open'));
  expect(handleDoorOpen).toBeCalled();
});

test('triggers doCloseDoor callback when button is clicked', () => {
  const handleDoorClose = jest.fn();
  const root = render(<DoorAccordion door={makeDoor()} onCloseClick={handleDoorClose} />);
  userEvent.click(root.getByText('Close'));
  expect(handleDoorClose).toBeCalled();
});
