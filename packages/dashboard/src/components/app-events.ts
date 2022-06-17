import { Dispenser, Door, Ingestor, Lift } from 'api-client';
import { Subject } from 'rxjs';

export const AppEvents = {
  doorSelect: new Subject<Door>(),
  liftSelect: new Subject<Lift>(),
  dispenserSelect: new Subject<Dispenser>(),
  ingestorSelect: new Subject<Ingestor>(),
  robotSelect: new Subject<[fleetName: string, robotName: string]>(),
};
