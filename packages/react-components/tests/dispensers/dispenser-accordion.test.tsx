import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { render } from '@testing-library/react';
import React from 'react';
import { DispenserAccordion } from '../../lib';

const baseDispenser: RomiCore.DispenserState = {
  guid: 'test',
  mode: RomiCore.DispenserState.IDLE,
  request_guid_queue: [],
  seconds_remaining: 0,
  time: { sec: 0, nanosec: 0 },
};

test('smoke test', () => {
  render(<DispenserAccordion dispenserState={baseDispenser} />);
});
