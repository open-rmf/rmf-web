import { Door as RmfDoor } from 'rmf-models/ros/rmf_building_map_msgs/msg';
import { DoorMode as RmfDoorMode } from 'rmf-models/ros/rmf_door_msgs/msg';
import { describe, expect, it } from 'vitest';

import { doorModeToString, doorTypeToString } from './door-utils';

describe('door utils', () => {
  it('doorModeToString', () => {
    expect(doorModeToString(RmfDoorMode.MODE_OPEN)).toEqual('OPEN');
    expect(doorModeToString(RmfDoorMode.MODE_CLOSED)).toEqual('CLOSED');
    expect(doorModeToString(RmfDoorMode.MODE_MOVING)).toEqual('MOVING');
    expect(doorModeToString(-1)).toEqual('UNKNOWN');
  });

  it('doorTypeToString', () => {
    expect(doorTypeToString(RmfDoor.DOOR_TYPE_DOUBLE_SLIDING)).toEqual('Double Sliding');
    expect(doorTypeToString(RmfDoor.DOOR_TYPE_DOUBLE_SWING)).toEqual('Double Swing');
    expect(doorTypeToString(RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE)).toEqual('Double Telescope');
    expect(doorTypeToString(RmfDoor.DOOR_TYPE_SINGLE_SLIDING)).toEqual('Single Sliding');
    expect(doorTypeToString(RmfDoor.DOOR_TYPE_SINGLE_SWING)).toEqual('Single Swing');
    expect(doorTypeToString(RmfDoor.DOOR_TYPE_SINGLE_TELESCOPE)).toEqual('Single Telescope');
    expect(doorTypeToString(-1)).toEqual('Unknown (-1)');
  });
});
