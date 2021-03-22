import type {
  AffineImage as BaseAffineImage,
  BuildingMap as BaseBuildingMap,
  Level as BaseLevel,
} from '@osrf/romi-js-core-interfaces';
import { DispenserState as DispenserStateModel } from './generated/dispenser-state';
import { DoorState as DoorStateModel } from './generated/door-state';
import { FleetState as FleetStateModel } from './generated/fleet-state';
import { IngestorState as IngestorStateModel } from './generated/ingestor-state';
import { LiftState as LiftStateModel } from './generated/lift-state';
import { DispenserState as RmfDispenserState } from './generated/ros/rmf_dispenser_msgs/msg/DispenserState';
import { DoorState as RmfDoorState } from './generated/ros/rmf_door_msgs/msg/DoorState';
import { FleetState as RmfFleetState } from './generated/ros/rmf_fleet_msgs/msg/FleetState';
import { IngestorState as RmfIngestorState } from './generated/ros/rmf_ingestor_msgs/msg/IngestorState';
import { LiftState as RmfLiftState } from './generated/ros/rmf_lift_msgs/msg/LiftState';
import { TaskSummary as RmfTaskSummary } from './generated/ros/rmf_task_msgs/msg/TaskSummary';
import { TaskSummary as TaskSummaryModel } from './generated/task-summary';

export { DispenserHealth } from './generated/dispenser-health';
export { DoorHealth } from './generated/door-health';
export { IngestorHealth } from './generated/ingestor-health';
export { LiftHealth } from './generated/lift-health';
export { RobotHealth } from './generated/robot-health';

type RmfJsonWrapperT<ModelT, RmfT> = Omit<ModelT, 'data'> & { data: RmfT };

export type DoorState = RmfJsonWrapperT<DoorStateModel, RmfDoorState>;
export type LiftState = RmfJsonWrapperT<LiftStateModel, RmfLiftState>;
export type DispenserState = RmfJsonWrapperT<DispenserStateModel, RmfDispenserState>;
export type IngestorState = RmfJsonWrapperT<IngestorStateModel, RmfIngestorState>;
export type FleetState = RmfJsonWrapperT<FleetStateModel, RmfFleetState>;
export type TaskSummary = RmfJsonWrapperT<TaskSummaryModel, RmfTaskSummary>;

export interface AffineImage extends Omit<BaseAffineImage, 'data'> {
  data: string;
}

export interface Level extends Omit<BaseLevel, 'images'> {
  images: AffineImage[];
}

export interface BuildingMap extends Omit<BaseBuildingMap, 'levels'> {
  levels: Level[];
}
