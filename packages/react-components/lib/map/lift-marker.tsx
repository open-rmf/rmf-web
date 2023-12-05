import type { LiftState } from 'api-client';
import { LiftState as RmfLiftState } from 'rmf-models';

// Gets the text to insert to the lift, the text depend on the current mode, motion state and the
// current and destination floor of the lift.
export const getLiftModeText = (liftState: LiftState): string => {
  if (!liftState.current_mode) {
    return 'UNKNOWN';
  }
  switch (liftState.current_mode) {
    case RmfLiftState.MODE_FIRE:
      return 'FIRE!';
    case RmfLiftState.MODE_EMERGENCY:
      return 'EMERGENCY!';
    case RmfLiftState.MODE_OFFLINE:
      return 'OFFLINE';
    default:
      return 'NORMAL';
  }
};
