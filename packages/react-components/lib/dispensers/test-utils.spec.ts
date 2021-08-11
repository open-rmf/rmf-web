import * as RmfModels from 'rmf-models';
import { Dispenser } from './utils';

export function allStateModes(): number[] {
  return [
    RmfModels.DispenserState.IDLE,
    RmfModels.DispenserState.OFFLINE,
    RmfModels.DispenserState.BUSY,
    -1,
  ];
}

export function makeDispenserState(
  dispenserState?: Partial<RmfModels.DispenserState>,
): RmfModels.DispenserState {
  return {
    guid: 'test',
    mode: RmfModels.DispenserState.IDLE,
    request_guid_queue: [],
    seconds_remaining: 0,
    time: { sec: 0, nanosec: 0 },
    ...dispenserState,
  };
}

export function makeDispenser(dispenser?: Dispenser) {
  return {
    guid: 'test',
    ...dispenser,
  };
}
