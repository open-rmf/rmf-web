import * as RomiCore from '@osrf/romi-js-core-interfaces';

export const doorStates: { [key: string]: RomiCore.DoorState } = {
  Door1: {
    door_name: 'Door1',
    current_mode: { value: RomiCore.DoorMode.MODE_OPEN },
    door_time: { sec: 0, nanosec: 0 },
  },
  Door2: {
    door_name: 'Door2',
    current_mode: { value: RomiCore.DoorMode.MODE_CLOSED },
    door_time: { sec: 0, nanosec: 0 },
  },
};

export default doorStates;
