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
} from 'rmf-models';
import { io, Socket } from 'socket.io-client';

export class SioClient {
  public sio: Socket;

  constructor(...args: Parameters<typeof io>) {
    this.sio = io(...args);
  }

  subscribe<T>(path: string, cb: (resp: T) => void): void {
    this.sio.emit('subscribe', { path });
    this.sio.on(path, (msg: T) => cb(msg));
  }

  subscribeBuildingMap(cb: (buildingMap: BuildingMap) => void): void {
    return this.subscribe<BuildingMap>(`/building_map`, cb);
  }

  subscribeDoorState(doorName: string, cb: (doorState: DoorState) => void): void {
    return this.subscribe<DoorState>(`/doors/${doorName}/state`, cb);
  }

  subscribeDoorHealth(doorName: string, cb: (doorHealth: DoorHealth) => void): void {
    return this.subscribe<DoorHealth>(`/doors/${doorName}/health`, cb);
  }

  subscribeLiftState(liftName: string, cb: (liftState: LiftState) => void): void {
    return this.subscribe<LiftState>(`/lifts/${liftName}/state`, cb);
  }

  subscribeLiftHealth(liftName: string, cb: (liftHealth: LiftHealth) => void): void {
    return this.subscribe<LiftHealth>(`/doors/${liftName}/health`, cb);
  }

  subscribeDispenserState(guid: string, cb: (dispenserState: DispenserState) => void): void {
    return this.subscribe<DispenserState>(`/dispensers/${guid}/state`, cb);
  }

  subscribeDispenserHealth(guid: string, cb: (dispenserHealth: DispenserHealth) => void): void {
    return this.subscribe<DispenserHealth>(`/dispensers/${guid}/health`, cb);
  }

  subscribeIngestorState(guid: string, cb: (ingestorState: IngestorState) => void): void {
    return this.subscribe<IngestorState>(`/ingestors/${guid}/state`, cb);
  }

  subscribeIngestorHealth(guid: string, cb: (ingestorHealth: IngestorHealth) => void): void {
    return this.subscribe<IngestorHealth>(`/ingestors/${guid}/health`, cb);
  }

  subscribeFleetState(name: string, cb: (fleetState: FleetState) => void): void {
    return this.subscribe<FleetState>(`/fleets/${name}/state`, cb);
  }

  subscribeRobotHealth(
    fleetName: string,
    robotName: string,
    cb: (robotHealth: RobotHealth) => void,
  ): void {
    return this.subscribe<RobotHealth>(`/fleets/${fleetName}/${robotName}/health`, cb);
  }
}

export * from './openapi';
export * from './openapi/models';
