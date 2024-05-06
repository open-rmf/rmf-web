import Debug from 'debug';
import { io, Socket } from 'socket.io-client';
import {
  ApiServerModelsTortoiseModelsAlertsAlertLeaf,
  ApiServerModelsTortoiseModelsBeaconsBeaconStateLeaf as BeaconState,
  BuildingMap,
  DeliveryAlert,
  DispenserHealth,
  DispenserState,
  DoorHealth,
  DoorState,
  FireAlarmTriggerState,
  FleetState,
  IngestorHealth,
  IngestorState,
  LiftHealth,
  LiftState,
  TaskEventLog,
  TaskState,
} from './openapi';

type Alert = ApiServerModelsTortoiseModelsAlertsAlertLeaf;

const debug = Debug('rmf-client');

// https://stackoverflow.com/questions/52667959/what-is-the-purpose-of-bivariancehack-in-typescript-types
export type Listener<T = unknown> = { bivarianceHack(resp: T): void }['bivarianceHack'];
export interface Subscription {
  room: string;
  listener: Listener;
}

export class SioClient {
  public sio: Socket;

  constructor(...args: Parameters<typeof io>) {
    this.sio = io(...args);
  }

  subscribe<T>(room: string, listener: Listener<T>): Subscription {
    this.sio.emit('subscribe', { room });
    debug(`subscribed to ${room}`);
    this.sio.on(room, listener);
    return { room, listener };
  }

  unsubscribe(sub: Subscription): void {
    this.sio.emit('unsubscribe', { room: sub.room });
    debug(`unsubscribed to ${sub.room}`);
    this.sio.off(sub.room, sub.listener);
  }

  subscribeBeaconState(listener: Listener<BeaconState>): Subscription {
    return this.subscribe<BeaconState>('/beacons', listener);
  }

  subscribeBuildingMap(listener: Listener<BuildingMap>): Subscription {
    return this.subscribe<BuildingMap>(`/building_map`, listener);
  }

  subscribeDoorState(doorName: string, listener: Listener<DoorState>): Subscription {
    return this.subscribe<DoorState>(`/doors/${doorName}/state`, listener);
  }

  subscribeDoorHealth(doorName: string, listener: Listener<DoorHealth>): Subscription {
    return this.subscribe<DoorHealth>(`/doors/${doorName}/health`, listener);
  }

  subscribeLiftState(liftName: string, listener: Listener<LiftState>): Subscription {
    return this.subscribe<LiftState>(`/lifts/${liftName}/state`, listener);
  }

  subscribeLiftHealth(liftName: string, listener: Listener<LiftHealth>): Subscription {
    return this.subscribe<LiftHealth>(`/doors/${liftName}/health`, listener);
  }

  subscribeDispenserState(guid: string, listener: Listener<DispenserState>): Subscription {
    return this.subscribe<DispenserState>(`/dispensers/${guid}/state`, listener);
  }

  subscribeDispenserHealth(guid: string, listener: Listener<DispenserHealth>): Subscription {
    return this.subscribe<DispenserHealth>(`/dispensers/${guid}/health`, listener);
  }

  subscribeIngestorState(guid: string, listener: Listener<IngestorState>): Subscription {
    return this.subscribe<IngestorState>(`/ingestors/${guid}/state`, listener);
  }

  subscribeIngestorHealth(guid: string, listener: Listener<IngestorHealth>): Subscription {
    return this.subscribe<IngestorHealth>(`/ingestors/${guid}/health`, listener);
  }

  subscribeFleetState(name: string, listener: Listener<FleetState>): Subscription {
    return this.subscribe<FleetState>(`/fleets/${name}/state`, listener);
  }

  subscribeTaskState(taskId: string, listener: Listener<TaskState>): Subscription {
    return this.subscribe<TaskState>(`/tasks/${taskId}/state`, listener);
  }

  subscribeTaskLogs(taskId: string, listener: Listener<TaskEventLog>): Subscription {
    return this.subscribe<TaskEventLog>(`/tasks/${taskId}/log`, listener);
  }

  subscribeAlerts(listener: Listener<Alert>): Subscription {
    return this.subscribe<Alert>(`/alerts`, listener);
  }

  subscribeDeliveryAlerts(listener: Listener<DeliveryAlert>): Subscription {
    return this.subscribe<DeliveryAlert>(`/delivery_alerts`, listener);
  }

  subscribeFireAlarmTrigger(listener: Listener<FireAlarmTriggerState>): Subscription {
    return this.subscribe<FireAlarmTriggerState>('/fire_alarm_trigger', listener);
  }
}

export * from './openapi';
