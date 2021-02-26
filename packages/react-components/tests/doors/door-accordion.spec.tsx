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

  it('renders basic door information', () => {
    const door = makeDoor({
      name: 'test_door',
      door_type: RomiCore.Door.DOOR_TYPE_SINGLE_SWING,
      motion_direction: 1,
    });
    const doorState = makeDoorState({
      door_name: 'test_door',
      current_mode: { value: RomiCore.DoorMode.MODE_CLOSED },
    });
    const accordion = render(<DoorAccordion door={door} doorState={doorState} />);
    expect(accordion.queryAllByText('test_door').length).toBeTruthy();
    expect(accordion.queryByText('Single Swing')).toBeTruthy();
    expect(accordion.queryByText('Clockwise')).toBeTruthy();
    expect(accordion.queryByText('1.571')).toBeTruthy();
    expect(accordion.queryByText('(0.000, 0.000)')).toBeTruthy();
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
