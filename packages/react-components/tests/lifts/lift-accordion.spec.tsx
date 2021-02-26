import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { cleanup, render } from '@testing-library/react';
import React from 'react';
import { LiftAccordion } from '../../lib';
import { allLiftMotion, makeLift, makeLiftState } from './test-utils';

it('smoke test with different lift motion', () => {
  allLiftMotion()
    .map((motion) =>
      makeLiftState({
        motion_state: motion,
      }),
    )
    .forEach((state) => {
      render(<LiftAccordion lift={makeLift()} liftState={state} />);
      cleanup();
    });
});

it('smoke test without lift state', () => {
  render(<LiftAccordion lift={makeLift()} />);
});

it('renders basic lift information', () => {
  const lift = makeLift({
    name: 'test_lift',
    doors: [
      {
        name: 'door',
        door_type: RomiCore.Door.DOOR_TYPE_DOUBLE_TELESCOPE,
        motion_direction: 1,
        motion_range: Math.PI / 2,
        v1_x: -1,
        v1_y: -1,
        v2_x: 1,
        v2_y: -1,
      },
    ],
  });
  const liftState = makeLiftState({
    lift_name: 'test_lift',
    available_floors: ['L1', 'L2'],
    available_modes: Uint8Array.from([RomiCore.LiftState.MODE_AGV]),
    current_floor: 'L1',
    current_mode: RomiCore.LiftState.MODE_AGV,
    destination_floor: 'L1',
    door_state: RomiCore.LiftState.DOOR_CLOSED,
    motion_state: RomiCore.LiftState.MOTION_STOPPED,
    session_id: 'test_session',
  });
  const accordion = render(<LiftAccordion lift={lift} liftState={liftState} />);
  expect(accordion.queryAllByText('test_lift').length).toBeTruthy();
  expect(accordion.queryByText('(0.000, 0.000)')).toBeTruthy();
  expect(accordion.queryAllByText('L1')).toHaveSize(3); // destination floor, current floor and available floors
  expect(accordion.queryByText('L2')).toBeTruthy();
  expect(accordion.queryAllByText('AGV')).toHaveSize(2); // current mode and available modes
  expect(accordion.queryByText('Closed')).toBeTruthy();
  expect(accordion.queryByText('Stopped')).toBeTruthy();
});
