import type { Dispenser, DispenserState } from 'api-client';
import { DispenserState as RmfDispenserState } from 'rmf-models';

export function allStateModes(): number[] {
  return [RmfDispenserState.IDLE, RmfDispenserState.OFFLINE, RmfDispenserState.BUSY, -1];
}

export function makeDispenserState(dispenserState?: Partial<DispenserState>): DispenserState {
  return {
    guid: 'test',
    mode: RmfDispenserState.IDLE,
    request_guid_queue: [],
    seconds_remaining: 0,
    time: { sec: 0, nanosec: 0 },
    ...dispenserState,
  };
}

export function makeDispenser(dispenser?: Dispenser): Dispenser {
  return {
    guid: 'test',
    ...dispenser,
  };
}
