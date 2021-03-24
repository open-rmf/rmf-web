import { io, Socket } from 'socket.io-client';
import * as mdl from './models';

export type Topic =
  | 'door_states'
  | 'door_health'
  | 'lift_states'
  | 'lift_health'
  | 'dispenser_states'
  | 'dispenser_health'
  | 'ingestor_state'
  | 'ingestor_health'
  | 'fleet_states'
  | 'robot_health'
  | 'task_summaries'
  | 'building_map';

export interface ApiClient extends Socket {
  on(event: 'subscribe', listener: (resp: string) => void): this;
  on(event: 'door_states', listener: (resp: mdl.DoorState) => void): this;
  on(event: 'door_health', listener: (resp: mdl.DoorHealth) => void): this;
  on(event: 'lift_states', listener: (resp: mdl.LiftState) => void): this;
  on(event: 'lift_health', listener: (resp: mdl.LiftHealth) => void): this;
  on(event: 'dispenser_states', listener: (resp: mdl.DispenserState) => void): this;
  on(event: 'dispenser_health', listener: (resp: mdl.DispenserHealth) => void): this;
  on(event: 'ingestor_states', listener: (resp: mdl.IngestorState) => void): this;
  on(event: 'ingestor_health', listener: (resp: mdl.IngestorHealth) => void): this;
  on(event: 'fleet_states', listener: (resp: mdl.FleetState) => void): this;
  on(event: 'robot_health', listener: (resp: mdl.RobotHealth) => void): this;
  on(event: 'task_summaries', listener: (resp: mdl.TaskSummary) => void): this;
  on(event: 'building_map', listener: (resp: mdl.BuildingMap) => void): this;
  on(event: string, listener: Function): this; // eslint-disable-line @typescript-eslint/ban-types

  emit(event: 'subscribe', topic: Topic): this;
  emit(event: string, ...args: any[]): this; // eslint-disable-line @typescript-eslint/no-explicit-any
}

export function makeClient(...args: Parameters<typeof io>): ApiClient {
  return io(...args) as ApiClient;
}
