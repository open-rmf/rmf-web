import * as RomiCore from '@osrf/romi-js-core-interfaces';

export const dispenserStates: { [key: string]: RomiCore.DispenserState } = {
  Dispenser1: {
    time: { sec: 0, nanosec: 0 },
    guid: 'dispenser_1',
    mode: 1,
    request_guid_queue: ['request_0', 'request_1'],
    seconds_remaining: 0,
  },
  Dispenser2: {
    time: { sec: 0, nanosec: 0},
    guid: 'dispenser_2',
    mode: 1,
    request_guid_queue: ['request_2'],
    seconds_remaining: 0,
  },
};

export default dispenserStates;
