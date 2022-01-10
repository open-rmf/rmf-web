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
} from './openapi';

const debug = Debug('rmf-client');

// https://stackoverflow.com/questions/52667959/what-is-the-purpose-of-bivariancehack-in-typescript-types
export type Listener<T = unknown> = { bivarianceHack(resp: T): void }['bivarianceHack'];
export type Subscription = Listener;

export class SioClient {
  public sio: Socket;
  private _subscriptions = new Map<Listener, string>();

  constructor(...args: Parameters<typeof io>) {
    this.sio = io(...args);
  }

  subscribe<T>(room: string, listener: Listener<T>): Listener<T> {
    this.sio.emit('subscribe', { room });
    this.sio.on(room, listener);
    this._subscriptions.set(listener, room);
    debug(`subscribed to ${room}`);
    return listener;
  }

  unsubscribe(listener: Listener): void {
    const room = this._subscriptions.get(listener);
    if (room) {
      this.sio.emit('unsubscribe', { room });
      this.sio.off(room, listener);
      this._subscriptions.delete(listener);
      debug(`unsubscribed to ${room}`);
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
}

export * from './openapi';
