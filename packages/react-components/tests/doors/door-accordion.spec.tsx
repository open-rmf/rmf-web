import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { cleanup, render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorAccordion } from '../../lib';
import { allDoorModes, allDoorTypes, makeDoor, makeDoorState } from './test-utils';

describe('Door accordion', () => {
  it('smoke test with different door types', () => {
    allDoorTypes()
      .map((type) =>
        makeDoor({
          door_type: type,
        }),
      )
      .forEach((door) => {
        render(<DoorAccordion door={door} />);
        cleanup();
      });
  });

  it('smoke test with different door modes', () => {
    allDoorModes()
      .map((mode) => makeDoorState({ current_mode: mode }))
      .forEach((state) => {
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
