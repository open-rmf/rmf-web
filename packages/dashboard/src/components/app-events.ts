import {
  ApiServerModelsTortoiseModelsAlertsAlertLeaf,
  Dispenser,
  Door,
  Ingestor,
  Lift,
  TaskState,
} from 'api-client';
import { Subject } from 'rxjs';

type Alert = ApiServerModelsTortoiseModelsAlertsAlertLeaf;

export const AppEvents = {
  doorSelect: new Subject<Door | null>(),
  liftSelect: new Subject<Lift | null>(),
  dispenserSelect: new Subject<Dispenser | null>(),
  ingestorSelect: new Subject<Ingestor | null>(),
  robotSelect: new Subject<[fleetName: string, robotName: string] | null>(),
  taskSelect: new Subject<TaskState | null>(),
  refreshTaskQueueTableCount: new Subject<number>(),
  refreshAlertCount: new Subject<number>(),
  alertListOpenedAlert: new Subject<Alert | null>(),
};
