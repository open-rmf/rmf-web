import * as RomiCore from '@osrf/romi-js-core-interfaces';

export const dispenserStates: { [key: string]: RomiCore.DispenserState } = {
  Dispenser0: {
    time: { sec: 0, nanosec: 0 },
    guid: 'Dispenser0',
    mode: 0,
    request_guid_queue: [],
    seconds_remaining: 0,
  },
  Dispenser1: {
    time: { sec: 0, nanosec: 0},
    guid: 'Dispenser1',
    mode: 1,
    request_guid_queue: ['Request0', 'Request1', 'Request2', 'Request3', 
        'Request4', 'Request5', 'Request6', 'Request7', 'Request8', 
        'Request9'],
    seconds_remaining: 420,
  },
  Dispenser2: {
    time: { sec: 0, nanosec: 0},
    guid: 'Dispenser2',
    mode: 2,
    request_guid_queue: [],
    seconds_remaining: 0,
  },
};

export default dispenserStates;
