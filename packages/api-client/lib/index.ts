import Debug from 'debug';
import { io, Socket } from 'socket.io-client';
import {
  BuildingMap,
  DispenserHealth,
  DispenserState,
  DoorHealth,
  DoorState,
  FleetState,
  IngestorHealth,
  IngestorState,
  LiftHealth,
  LiftState,
  RobotHealth,
  TaskSummary,
} from './openapi';

const debug = Debug('rmf-client');

// https://stackoverflow.com/questions/52667959/what-is-the-purpose-of-bivariancehack-in-typescript-types
export type Listener<T = unknown> = { bivarianceHack(resp: T): void }['bivarianceHack'];
export type Subscription = Listener;

export class SioClient {
  public sio: Socket;
  private _subscriptions: Record<string, number> = {};
  private _listenerRoom = new Map<Listener, string>();

  constructor(...args: Parameters<typeof io>) {
    this.sio = io(...args);
  }

  subscribe<T>(room: string, listener: Listener<T>): Listener<T> {
    const subs = this._subscriptions[room] || 0;
    if (subs === 0) {
      this.sio.emit('subscribe', { room });
      debug(`subscribed to ${room}`);
    } else {
      debug(`reusing previous subscription to ${room}`);
    }
    this.sio.on(room, listener);
    this._subscriptions[room] = subs + 1;
    this._listenerRoom.set(listener, room);
    return listener;
  }

  unsubscribe(listener: Listener): void {
    const room = this._listenerRoom.get(listener);
    if (room) {
      const count = this._subscriptions[room] || 0;
      if (count - 1 <= 0) {
        this.sio.emit('unsubscribe', { room });
        delete this._subscriptions[room];
        debug(`unsubscribed to ${room}`);
      } else {
        this._subscriptions[room] = count - 1;
        debug(`skipping unsubscribe to ${room} because there are still ${count - 1} subscribers`);
      }
      this.sio.off(room, listener);
      this._listenerRoom.delete(listener);
    } else {
      debug('fail to unsubscribe, listener not found in list of subscriptions');
    }
  }

  subscribeBuildingMap(listener: Listener<BuildingMap>): Listener<BuildingMap> {
    return this.subscribe<BuildingMap>(`/building_map`, listener);
  }

  subscribeDoorState(doorName: string, listener: Listener<DoorState>): Listener<DoorState> {
    return this.subscribe<DoorState>(`/doors/${doorName}/state`, listener);
  }

  subscribeDoorHealth(doorName: string, listener: Listener<DoorHealth>): Listener<DoorHealth> {
    return this.subscribe<DoorHealth>(`/doors/${doorName}/health`, listener);
  }

  subscribeLiftState(liftName: string, listener: Listener<LiftState>): Listener<LiftState> {
    return this.subscribe<LiftState>(`/lifts/${liftName}/state`, listener);
  }

  subscribeLiftHealth(liftName: string, listener: Listener<LiftHealth>): Listener<LiftHealth> {
    return this.subscribe<LiftHealth>(`/doors/${liftName}/health`, listener);
  }

  subscribeDispenserState(
    guid: string,
    listener: Listener<DispenserState>,
  ): Listener<DispenserState> {
    return this.subscribe<DispenserState>(`/dispensers/${guid}/state`, listener);
  }

  subscribeDispenserHealth(
    guid: string,
    listener: Listener<DispenserHealth>,
  ): Listener<DispenserHealth> {
    return this.subscribe<DispenserHealth>(`/dispensers/${guid}/health`, listener);
  }

  subscribeIngestorState(guid: string, listener: Listener<IngestorState>): Listener<IngestorState> {
    return this.subscribe<IngestorState>(`/ingestors/${guid}/state`, listener);
  }

  subscribeIngestorHealth(
    guid: string,
    listener: Listener<IngestorHealth>,
  ): Listener<IngestorHealth> {
    return this.subscribe<IngestorHealth>(`/ingestors/${guid}/health`, listener);
  }

  subscribeFleetState(name: string, listener: Listener<FleetState>): Listener<FleetState> {
    return this.subscribe<FleetState>(`/fleets/${name}/state`, listener);
  }

  subscribeRobotHealth(
    fleetName: string,
    robotName: string,
    listener: Listener<RobotHealth>,
  ): Listener<RobotHealth> {
    return this.subscribe<RobotHealth>(`/fleets/${fleetName}/${robotName}/health`, listener);
  }

  subscribeTaskSummary(taskId: string, listener: Listener<TaskSummary>): Listener<TaskSummary> {
    const encoded = taskId.replace('/', '__');
    return this.subscribe<TaskSummary>(`/tasks/${encoded}/summary`, listener);
  }
}

export * from './openapi';
