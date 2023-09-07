import {
  ApiServerModelsTortoiseModelsAlertsAlertLeaf as Alert,
  Dispenser,
  Door,
  Ingestor,
  Lift,
  TaskState,
} from 'api-client';
import { BehaviorSubject, ReplaySubject, Subject } from 'rxjs';

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
  mapCenter: new BehaviorSubject<[number, number]>([0, 0]),
};
