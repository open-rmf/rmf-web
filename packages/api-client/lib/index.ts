import { io as _io, Socket } from 'socket.io-client';
import * as mdl from './models';

export type MessageType = {
  door_states: mdl.DoorState;
  door_health: mdl.DoorHealth;
  lift_states: mdl.LiftState;
  lift_health: mdl.LiftHealth;
  dispenser_states: mdl.DispenserState;
  dispenser_health: mdl.DispenserHealth;
  ingestor_states: mdl.IngestorState;
  ingestor_health: mdl.IngestorHealth;
  fleet_states: mdl.FleetState;
  robot_health: mdl.RobotHealth;
  task_summaries: mdl.TaskSummary;
  building_map: mdl.BuildingMap;
};

export type Topic = keyof MessageType;

export interface SioClient extends Socket {
  on(event: 'subscribe', listener: (resp: string) => void): this;
  on(event: 'door_states', listener: (resp: MessageType['door_states']) => void): this;
  on(event: 'door_health', listener: (resp: MessageType['door_health']) => void): this;
  on(event: 'lift_states', listener: (resp: MessageType['lift_states']) => void): this;
  on(event: 'lift_health', listener: (resp: MessageType['lift_health']) => void): this;
  on(event: 'dispenser_states', listener: (resp: MessageType['dispenser_states']) => void): this;
  on(event: 'dispenser_health', listener: (resp: MessageType['dispenser_health']) => void): this;
  on(event: 'ingestor_states', listener: (resp: MessageType['ingestor_states']) => void): this;
  on(event: 'ingestor_health', listener: (resp: MessageType['ingestor_health']) => void): this;
  on(event: 'fleet_states', listener: (resp: MessageType['fleet_states']) => void): this;
  on(event: 'robot_health', listener: (resp: MessageType['robot_health']) => void): this;
  on(event: 'task_summaries', listener: (resp: MessageType['task_summaries']) => void): this;
  on(event: 'building_map', listener: (resp: MessageType['building_map']) => void): this;
  on(event: string, listener: Function): this; // eslint-disable-line @typescript-eslint/ban-types

  emit(event: 'subscribe', topic: Topic): this;
  emit(event: string, ...args: any[]): this; // eslint-disable-line @typescript-eslint/no-explicit-any
}

export function io(...args: Parameters<typeof _io>): SioClient {
  return _io(...args) as SioClient;
}

export * from './models';
export * from './openapi';
