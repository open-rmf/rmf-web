import * as RmfModels from 'rmf-models';
import { io as _io, Socket } from 'socket.io-client';

export type MessageType = {
  door_states: RmfModels.DoorState;
  door_health: RmfModels.DoorHealth;
  lift_states: RmfModels.LiftState;
  lift_health: RmfModels.LiftHealth;
  dispenser_states: RmfModels.DispenserState;
  dispenser_health: RmfModels.DispenserHealth;
  ingestor_states: RmfModels.IngestorState;
  ingestor_health: RmfModels.IngestorHealth;
  fleet_states: RmfModels.FleetState;
  robot_health: RmfModels.RobotHealth;
  task_summaries: RmfModels.TaskSummary;
  building_map: RmfModels.BuildingMap;
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

export * from './openapi';
