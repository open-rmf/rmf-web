import { DispenserState as RmfDispenserState } from 'rmf-models';

export function dispenserModeToString(mode: number): string {
  switch (mode) {
    case RmfDispenserState.IDLE:
      return 'IDLE';
    case RmfDispenserState.BUSY:
      return 'ONLINE';
    case RmfDispenserState.OFFLINE:
      return 'OFFLINE';
    default:
      return 'N/A';
  }
}
