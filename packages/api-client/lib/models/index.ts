export { DoorHealth } from './generated/door-health';
export { DoorState } from './generated/door-state';
export { LiftState } from './generated/lift-state';
export { LiftHealth } from './generated/lift-health';
export { DispenserState } from './generated/dispenser-state';
export { DispenserHealth } from './generated/dispenser-health';
export { IngestorState } from './generated/ingestor-state';
export { IngestorHealth } from './generated/ingestor-health';
export { FleetState } from './generated/fleet-state';
export { RobotHealth } from './generated/robot-health';

import type {
  AffineImage as BaseAffineImage,
  BuildingMap as BaseBuildingMap,
  Level as BaseLevel,
} from '@osrf/romi-js-core-interfaces';

export interface AffineImage extends Omit<BaseAffineImage, 'data'> {
  data: string;
}

export interface Level extends Omit<BaseLevel, 'images'> {
  images: AffineImage[];
}

export interface BuildingMap extends Omit<BaseBuildingMap, 'levels'> {
  levels: Level[];
}
