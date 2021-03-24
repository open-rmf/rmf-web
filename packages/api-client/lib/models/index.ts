import type {
  AffineImage as BaseAffineImage,
  BuildingMap as BaseBuildingMap,
  Level as BaseLevel,
} from '@osrf/romi-js-core-interfaces';
export { DispenserHealth } from './generated/dispenser-health';
export { DoorHealth } from './generated/door-health';
export { IngestorHealth } from './generated/ingestor-health';
export { LiftHealth } from './generated/lift-health';
export { RobotHealth } from './generated/robot-health';
export { DispenserState } from './generated/ros/rmf_dispenser_msgs/msg/DispenserState';
export { DoorState } from './generated/ros/rmf_door_msgs/msg/DoorState';
export { FleetState } from './generated/ros/rmf_fleet_msgs/msg/FleetState';
export { IngestorState } from './generated/ros/rmf_ingestor_msgs/msg/IngestorState';
export { LiftState } from './generated/ros/rmf_lift_msgs/msg/LiftState';
export { TaskSummary } from './generated/ros/rmf_task_msgs/msg/TaskSummary';

export interface AffineImage extends Omit<BaseAffineImage, 'data'> {
  data: string;
}

export interface Level extends Omit<BaseLevel, 'images'> {
  images: AffineImage[];
}

export interface BuildingMap extends Omit<BaseBuildingMap, 'levels'> {
  levels: Level[];
}
