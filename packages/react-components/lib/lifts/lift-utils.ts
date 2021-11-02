import * as RmfModels from 'rmf-models';
import { LeafletContext } from 'react-leaflet';

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

export function onClickLift(lift: RmfModels.Lift, leafletMap?: LeafletContext) {
  leafletMap && leafletMap.map?.setView([lift.ref_y, lift.ref_x], 5.5);
}
