import { Dispenser, Door, Ingestor, Lift, TaskState } from 'api-client';
import { Subject } from 'rxjs';

export const AppEvents = {
  doorSelect: new Subject<Door>(),
  liftSelect: new Subject<Lift>(),
  dispenserSelect: new Subject<Dispenser>(),
  ingestorSelect: new Subject<Ingestor>(),
  robotSelect: new Subject<[fleetName: string, robotName: string]>(),
  taskSelect: new Subject<TaskState>(),
};
