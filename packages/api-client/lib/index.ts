import Debug from 'debug';
import { io, Socket } from 'socket.io-client';

import {
  AlertRequest,
  AlertResponse,
  BeaconState,
  BuildingMap,
  DeliveryAlert,
  DispenserState,
  DoorState,
  FleetState,
  IngestorState,
  LiftState,
  TaskEventLog,
  TaskStateOutput,
} from './openapi';

const debug = Debug('rmf-client');

// https://stackoverflow.com/questions/52667959/what-is-the-purpose-of-bivariancehack-in-typescript-types
export type Listener<T = unknown> = { bivarianceHack(resp: T): void }['bivarianceHack'];
export interface Subscription {
  room: string;
  listener: Listener;
}

export class SioClient {
  public sio: Socket;
  private _subscriptions: Record<string, number> = {};

  constructor(...args: Parameters<typeof io>) {
    this.sio = io(...args);
  }

  subscribe<T>(room: string, listener: Listener<T>): Subscription {
    const subs = this._subscriptions[room] || 0;
    if (subs === 0) {
      this.sio.emit('subscribe', { room });
      debug(`subscribed to ${room}`);
    } else {
      debug(`reusing previous subscription to ${room}`);
    }
    this.sio.on(room, listener);
    this._subscriptions[room] = subs + 1;
    return { room, listener };
  }

  unsubscribe(sub: Subscription): void {
    const subCount = this._subscriptions[sub.room] || 0;
    if (!subCount) {
      debug(`tried to unsubscribe from ${sub.room}, but no subscriptions exist`);
      // continue regardless
    }
    if (subCount <= 1) {
      this.sio.emit('unsubscribe', { room: sub.room });
      delete this._subscriptions[sub.room];
      debug(`unsubscribed to ${sub.room}`);
    } else {
      this._subscriptions[sub.room] = subCount - 1;
      debug(
        `skipping unsubscribe to ${sub.room} because there are still ${subCount - 1} subscribers`,
      );
    }
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

  subscribeLiftState(liftName: string, listener: Listener<LiftState>): Subscription {
    return this.subscribe<LiftState>(`/lifts/${liftName}/state`, listener);
  }

  subscribeDispenserState(guid: string, listener: Listener<DispenserState>): Subscription {
    return this.subscribe<DispenserState>(`/dispensers/${guid}/state`, listener);
  }

  subscribeIngestorState(guid: string, listener: Listener<IngestorState>): Subscription {
    return this.subscribe<IngestorState>(`/ingestors/${guid}/state`, listener);
  }

  subscribeFleetState(name: string, listener: Listener<FleetState>): Subscription {
    return this.subscribe<FleetState>(`/fleets/${name}/state`, listener);
  }

  subscribeTaskState(taskId: string, listener: Listener<TaskStateOutput>): Subscription {
    return this.subscribe<TaskStateOutput>(`/tasks/${taskId}/state`, listener);
  }

  subscribeTaskLogs(taskId: string, listener: Listener<TaskEventLog>): Subscription {
    return this.subscribe<TaskEventLog>(`/tasks/${taskId}/log`, listener);
  }

  subscribeAlertRequests(listener: Listener<AlertRequest>): Subscription {
    return this.subscribe<AlertRequest>(`/alerts/requests`, listener);
  }

  subscribeAlertResponses(listener: Listener<AlertResponse>): Subscription {
    return this.subscribe<AlertResponse>(`/alerts/responses`, listener);
  }

  subscribeDeliveryAlerts(listener: Listener<DeliveryAlert>): Subscription {
    return this.subscribe<DeliveryAlert>('/delivery_alerts', listener);
  }
}

export * from './openapi';
