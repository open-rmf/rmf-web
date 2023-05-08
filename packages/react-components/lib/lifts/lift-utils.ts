import { LiftRequest as RmfLiftRequest, LiftState as RmfLiftState } from 'rmf-models';

export function liftModeToString(liftMode?: number): string {
  if (liftMode === undefined) {
    return `Unknown (${liftMode})`;
  }
  switch (liftMode) {
    case RmfLiftState.MODE_AGV:
      return 'AGV';
    case RmfLiftState.MODE_EMERGENCY:
      return 'Emergency';
    case RmfLiftState.MODE_FIRE:
      return 'Fire';
    case RmfLiftState.MODE_HUMAN:
      return 'Human';
    case RmfLiftState.MODE_OFFLINE:
      return 'Offline';
    default:
      return `Unknown (${liftMode})`;
  }
}

export function doorStateToString(doorState?: number): string {
  if (doorState === undefined) return 'Unknown';

  switch (doorState) {
    case RmfLiftState.DOOR_OPEN:
      return 'OPEN';
    case RmfLiftState.DOOR_CLOSED:
      return 'CLOSED';
    case RmfLiftState.DOOR_MOVING:
      return 'MOVING';
    default:
      return `UNKNOWN (${doorState})`;
  }
}

export function motionStateToString(motionState?: number): string {
  if (motionState === undefined) return 'Unknown';

  switch (motionState) {
    case RmfLiftState.MOTION_DOWN:
      return 'Down';
    case RmfLiftState.MOTION_STOPPED:
      return 'Stopped';
    case RmfLiftState.MOTION_UP:
      return 'Up';
    default:
      return `Unknown (${motionState})`;
  }
}

export const requestModes = [
  RmfLiftRequest.REQUEST_AGV_MODE,
  RmfLiftRequest.REQUEST_HUMAN_MODE,
  RmfLiftRequest.REQUEST_END_SESSION,
];

export const requestModeStrings: Record<number, string> = {
  [RmfLiftRequest.REQUEST_END_SESSION]: 'End Session',
  [RmfLiftRequest.REQUEST_AGV_MODE]: 'AGV',
  [RmfLiftRequest.REQUEST_HUMAN_MODE]: 'Human',
};

export function requestModeToString(requestMode: number): string {
  return requestModeStrings[requestMode] || `Unknown (${requestMode})`;
}

export const requestDoorModes = [RmfLiftRequest.DOOR_OPEN, RmfLiftRequest.DOOR_CLOSED];

export const requestDoorModeStrings: Record<number, string> = {
  [RmfLiftRequest.DOOR_OPEN]: 'Open',
  [RmfLiftRequest.DOOR_CLOSED]: 'Closed',
};

export function requestDoorModeToString(requestDoorMode: number): string {
  return requestDoorModeStrings[requestDoorMode] || 'Unknown';
}
