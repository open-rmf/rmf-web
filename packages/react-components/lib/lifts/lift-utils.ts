import * as RmfModels from 'rmf-models';

export function liftModeToString(liftMode?: number): string {
  if (liftMode === undefined) {
    return `Unknown (${liftMode})`;
  }
  switch (liftMode) {
    case RmfModels.LiftState.MODE_AGV:
      return 'AGV';
    case RmfModels.LiftState.MODE_EMERGENCY:
      return 'Emergency';
    case RmfModels.LiftState.MODE_FIRE:
      return 'Fire';
    case RmfModels.LiftState.MODE_HUMAN:
      return 'Human';
    case RmfModels.LiftState.MODE_OFFLINE:
      return 'Offline';
    default:
      return `Unknown (${liftMode})`;
  }
}

export function doorStateToString(doorState?: number): string {
  if (doorState === undefined) return 'Unknown';

  switch (doorState) {
    case RmfModels.LiftState.DOOR_OPEN:
      return 'Open';
    case RmfModels.LiftState.DOOR_CLOSED:
      return 'Closed';
    case RmfModels.LiftState.DOOR_MOVING:
      return 'Moving';
    default:
      return `Unknown (${doorState})`;
  }
}

export function motionStateToString(motionState?: number): string {
  if (motionState === undefined) return 'Unknown';

  switch (motionState) {
    case RmfModels.LiftState.MOTION_DOWN:
      return 'Down';
    case RmfModels.LiftState.MOTION_STOPPED:
      return 'Stopped';
    case RmfModels.LiftState.MOTION_UP:
      return 'Up';
    default:
      return `Unknown (${motionState})`;
  }
}

export const requestModes = [
  RmfModels.LiftRequest.REQUEST_AGV_MODE,
  RmfModels.LiftRequest.REQUEST_HUMAN_MODE,
  RmfModels.LiftRequest.REQUEST_END_SESSION,
];

export const requestModeStrings: Record<number, string> = {
  [RmfModels.LiftRequest.REQUEST_END_SESSION]: 'End Session',
  [RmfModels.LiftRequest.REQUEST_AGV_MODE]: 'AGV',
  [RmfModels.LiftRequest.REQUEST_HUMAN_MODE]: 'Human',
};

export function requestModeToString(requestMode: number): string {
  return requestModeStrings[requestMode] || `Unknown (${requestMode})`;
}

export const requestDoorModes = [
  RmfModels.LiftRequest.DOOR_OPEN,
  RmfModels.LiftRequest.DOOR_CLOSED,
];

export const requestDoorModeStrings: Record<number, string> = {
  [RmfModels.LiftRequest.DOOR_OPEN]: 'Open',
  [RmfModels.LiftRequest.DOOR_CLOSED]: 'Closed',
};

export function requestDoorModeToString(requestDoorMode: number): string {
  return requestDoorModeStrings[requestDoorMode] || 'Unknown';
}

// table cell has padding of 16px left and 24px right respectively
// need to deduct 40px away from actual width
export const liftTableCellConfig = {
  // column width in percent
  liftName: 0.133,
  opMode: 0.227,
  currentFloor: 0.16,
  destination: 0.152,
  doorState: 0.148,
  // last column deduct 32px
  button: 0.18,
  // row height in pixels
  rowHeight: 31,
};
