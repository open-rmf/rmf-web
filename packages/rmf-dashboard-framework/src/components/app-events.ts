import {
  AlertRequest,
  Dispenser,
  Door,
  Ingestor,
  Level,
  Lift,
  TaskStateOutput as TaskState,
} from 'api-client';
import { BehaviorSubject, ReplaySubject, Subject } from 'rxjs';
import { Vector3 } from 'three';

export const AppEvents = {
  doorSelect: new Subject<[levelName: string, door: Door] | null>(),
  liftSelect: new Subject<Lift | null>(),
  dispenserSelect: new Subject<Dispenser | null>(),
  ingestorSelect: new Subject<Ingestor | null>(),
  robotSelect: new Subject<[fleetName: string, robotName: string] | null>(),
  taskSelect: new Subject<TaskState | null>(),
  refreshRobotApp: new Subject<void>(),
  refreshTaskApp: new Subject<void>(),
  refreshFavoriteTasks: new Subject<void>(),
  refreshTaskSchedule: new Subject<void>(),
  pushAlert: new Subject<AlertRequest | null>(),
  disabledLayers: new ReplaySubject<Record<string, boolean>>(),
  zoom: new BehaviorSubject<number | null>(null),
  cameraPosition: new BehaviorSubject<Vector3 | null>(null),
  zoomIn: new Subject<void>(),
  zoomOut: new Subject<void>(),
  levelSelect: new BehaviorSubject<Level | null>(null),
  justLoggedIn: new BehaviorSubject<boolean>(false),
  resetCamera: new Subject<[x: number, y: number, z: number, zoom: number]>(),
  loadingBackdrop: new Subject<boolean>(),
};
