import {
  TortoiseContribPydanticCreatorApiServerModelsTortoiseModelsAlertsAlertLeaf as Alert,
  Dispenser,
  Door,
  Ingestor,
  Level,
  Lift,
  TaskState,
} from 'api-client';
import { BehaviorSubject, ReplaySubject, Subject } from 'rxjs';
import { Vector3 } from 'three';

export const AppEvents = {
  doorSelect: new Subject<Door | null>(),
  liftSelect: new Subject<Lift | null>(),
  dispenserSelect: new Subject<Dispenser | null>(),
  ingestorSelect: new Subject<Ingestor | null>(),
  robotSelect: new Subject<[fleetName: string, robotName: string] | null>(),
  taskSelect: new Subject<TaskState | null>(),
  refreshTaskApp: new Subject<void>(),
  refreshAlert: new Subject<void>(),
  alertListOpenedAlert: new Subject<Alert | null>(),
  disabledLayers: new ReplaySubject<Record<string, boolean>>(),
  zoom: new BehaviorSubject<number | null>(null),
  cameraPosition: new BehaviorSubject<Vector3 | null>(null),
  zoomIn: new Subject<void>(),
  zoomOut: new Subject<void>(),
  mapCenter: new BehaviorSubject<[number, number]>([0, 0]),
  levelSelect: new BehaviorSubject<Level | null>(null),
};
