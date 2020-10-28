import * as RomiCore from '@osrf/romi-js-core-interfaces';

export function liftModeToString(liftMode: number): string {
  switch (liftMode) {
    case RomiCore.LiftState.MODE_AGV:
      return 'AGV';
    case RomiCore.LiftState.MODE_EMERGENCY:
      return 'Emergency';
    case RomiCore.LiftState.MODE_FIRE:
      return 'Fire';
    case RomiCore.LiftState.MODE_HUMAN:
      return 'Human';
    case RomiCore.LiftState.MODE_OFFLINE:
      return 'Offline';
    default:
      return `Unknown (${liftMode})`;
  }
}

export function doorStateToString(doorState: number): string {
  switch (doorState) {
    case RomiCore.LiftState.DOOR_OPEN:
      return 'Open';
    case RomiCore.LiftState.DOOR_CLOSED:
      return 'Closed';
    case RomiCore.LiftState.DOOR_MOVING:
      return 'Moving';
    default:
      return `Unknown (${doorState})`;
  }
}

export function motionStateToString(motionState: number): string {
  switch (motionState) {
    case RomiCore.LiftState.MOTION_DOWN:
      return 'Down';
    case RomiCore.LiftState.MOTION_STOPPED:
      return 'Stopped';
    case RomiCore.LiftState.MOTION_UP:
      return 'Up';
    default:
      return `Unknown (${motionState})`;
  }
}

export const requestModes = [
  RomiCore.LiftRequest.REQUEST_AGV_MODE,
  RomiCore.LiftRequest.REQUEST_HUMAN_MODE,
  RomiCore.LiftRequest.REQUEST_END_SESSION,
];

export const requestModeStrings: Record<number, string> = {
  [RomiCore.LiftRequest.REQUEST_END_SESSION]: 'End Session',
  [RomiCore.LiftRequest.REQUEST_AGV_MODE]: 'AGV',
  [RomiCore.LiftRequest.REQUEST_HUMAN_MODE]: 'Human',
};

export function requestModeToString(requestMode: number): string {
  return requestModeStrings[requestMode] || `Unknown (${requestMode})`;
}

export const requestDoorModes = [RomiCore.LiftRequest.DOOR_OPEN, RomiCore.LiftRequest.DOOR_CLOSED];

export const requestDoorModeStrings: Record<number, string> = {
  [RomiCore.LiftRequest.DOOR_OPEN]: 'Open',
  [RomiCore.LiftRequest.DOOR_CLOSED]: 'Closed',
};

export function requestDoorModeToString(requestDoorMode: number): string {
  return requestDoorModeStrings[requestDoorMode] || 'Unknown';
}
