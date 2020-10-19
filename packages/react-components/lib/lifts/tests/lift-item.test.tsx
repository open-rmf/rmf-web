import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { render } from '@testing-library/react';
import React from 'react';
import { LiftItem } from '..';
import { makeLift } from './test-utils';

function makeLiftComponent(
  lift?: Partial<RomiCore.Lift>,
  state?: Partial<RomiCore.LiftState>,
): JSX.Element {
  const state_: RomiCore.LiftState | undefined =
    state === undefined
      ? undefined
      : {
          lift_name: 'test',
          available_floors: ['test'],
          available_modes: Uint8Array.from([RomiCore.LiftState.MODE_AGV]),
          current_floor: 'test',
          current_mode: RomiCore.LiftState.MODE_AGV,
          destination_floor: 'test',
          door_state: RomiCore.LiftState.DOOR_OPEN,
          motion_state: RomiCore.LiftState.MOTION_STOPPED,
          session_id: 'test',
          lift_time: { sec: 0, nanosec: 0 },
          ...state,
        };

  return <LiftItem lift={makeLift(lift)} liftState={state_} />;
}

[
  {
    Component: makeLiftComponent(),
  },
  {
    Component: makeLiftComponent(undefined, { motion_state: RomiCore.LiftState.MOTION_STOPPED }),
    state: 'motion stopped',
  },
  {
    Component: makeLiftComponent(undefined, { motion_state: RomiCore.LiftState.MOTION_UP }),
    state: 'motion up',
  },
  {
    Component: makeLiftComponent(undefined, { motion_state: RomiCore.LiftState.MOTION_DOWN }),
    state: 'motion down',
  },
  {
    Component: makeLiftComponent(undefined, { motion_state: RomiCore.LiftState.MOTION_UNKNOWN }),
    state: 'motion unknown',
  },
  {
    Component: makeLiftComponent(undefined, { motion_state: -1 }),
    state: 'motion invalid',
  },
].map((params) => {
  test(`renders [state=${params.state}] correctly`, () => {
    const root = render(params.Component);
    expect(root.container).toMatchSnapshot();
  });
});
