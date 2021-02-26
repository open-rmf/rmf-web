import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { cleanup, render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorAccordion } from '../../lib';
import { makeDoor, makeDoorState } from './test-utils';

describe('Door accordion', () => {
  it('smoke test with different door types', () => {
    [
      makeDoor({
        door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING,
      }),
      makeDoor({
        door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_SWING,
      }),
      makeDoor({
        door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE,
      }),
      makeDoor({
        door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING,
      }),
      makeDoor({
        door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SWING,
      }),
      makeDoor({
        door_type: RomiCore.Door.DOOR_TYPE_SINGLE_TELESCOPE,
      }),
      makeDoor({
        door_type: RomiCore.Door.DOOR_TYPE_UNDEFINED,
      }),
      makeDoor({
        door_type: -1,
      }),
    ].forEach((door) => {
      render(<DoorAccordion door={door} />);
      cleanup();
    });
  });

  it('smoke test with different door modes', () => {
    [
      makeDoorState({
        current_mode: { value: RomiCore.DoorMode.MODE_CLOSED },
      }),
      makeDoorState({
        current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
      }),
      makeDoorState({
        current_mode: { value: RomiCore.DoorMode.MODE_MOVING },
      }),
      makeDoorState({
        current_mode: { value: -1 },
      }),
    ].forEach((state) => {
      render(<DoorAccordion door={makeDoor()} doorState={state} />);
      cleanup();
    });
  });

  it('smoke test with different motion direction', () => {
    [
      makeDoor({
        motion_direction: 1,
      }),
      makeDoor({
        motion_direction: -1,
      }),
      makeDoor({
        motion_direction: 100,
      }),
    ].forEach((door) => {
      render(<DoorAccordion door={door} />);
      cleanup();
    });
  });

  it('triggers door control dispatch when open door button is clicked', () => {
    const fakeOnClick = jasmine.createSpy();
    const root = render(<DoorAccordion door={makeDoor()} onDoorControlClick={fakeOnClick} />);

    userEvent.click(root.getByText('Open'));
    expect(fakeOnClick).toHaveBeenCalledWith(
      jasmine.anything(),
      jasmine.anything(),
      RomiCore.DoorMode.MODE_OPEN,
    );
  });

  it('triggers door control dispatch when close door button is clicked', () => {
    const fakeOnClick = jasmine.createSpy();
    const root = render(<DoorAccordion door={makeDoor()} onDoorControlClick={fakeOnClick} />);

    userEvent.click(root.getByText('Close'));
    expect(fakeOnClick).toHaveBeenCalledWith(
      jasmine.anything(),
      jasmine.anything(),
      RomiCore.DoorMode.MODE_CLOSED,
    );
  });
});
