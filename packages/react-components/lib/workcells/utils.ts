import { DispenserState as RmfDispenserState } from 'rmf-models/ros/rmf_dispenser_msgs/msg';

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
