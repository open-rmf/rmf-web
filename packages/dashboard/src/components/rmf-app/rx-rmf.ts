import { Subscription as RmfSubscription } from 'api-client';
import * as RmfModels from 'rmf-models';
import { BehaviorSubject, map, Observable, share } from 'rxjs';
import { RmfIngress } from './rmf-ingress';

/**
 * rxjs observable for RMF.
 */
export class RxRmf {
  rmfIngress: RmfIngress;
  private _buildingMap: Observable<RmfModels.BuildingMap | null>;
  private _doorStates: Record<string, Observable<RmfModels.DoorState | null>> = {};
  private _doorHealth: Record<string, Observable<RmfModels.DoorHealth | null>> = {};
  private _liftStates: Record<string, Observable<RmfModels.LiftState | null>> = {};
  private _liftHealth: Record<string, Observable<RmfModels.LiftHealth | null>> = {};
  private _dispenserStates: Record<string, Observable<RmfModels.DispenserState | null>> = {};
  private _dispenserHealth: Record<string, Observable<RmfModels.DispenserHealth | null>> = {};
  private _ingestorStates: Record<string, Observable<RmfModels.IngestorState | null>> = {};
  private _ingestorHealth: Record<string, Observable<RmfModels.IngestorHealth | null>> = {};
  private _fleetStates: Record<string, Observable<RmfModels.FleetState | null>> = {};
  private _robotStates: Record<
    string,
    Observable<{ fleet: string; robotState: RmfModels.RobotState } | null>
  > = {};
  private _robotHealth: Record<string, Observable<RmfModels.RobotHealth | null>> = {};
  private _taskSummaries: Record<string, Observable<RmfModels.TaskSummary | null>> = {};

  constructor(rmfIngress: RmfIngress) {
    this.rmfIngress = rmfIngress;

    this._buildingMap = this._wrapSioEvent<RmfModels.BuildingMap>(
      this.rmfIngress.sioClient.subscribeBuildingMap,
    );
  }

  private _getOrCreate<T>(obj: Record<string, T>, key: string, factory: () => T) {
    let result = obj[key];
    if (result !== undefined) {
      return result;
    }
    result = factory();
    obj[key] = result;
    return result;
  }

  private _wrapSioEvent<T>(sioSubFunc: (cb: (data: T) => void) => RmfSubscription) {
    return new Observable<T | null>((subscriber) => {
      const rmfSub = sioSubFunc((data) => subscriber.next(data));
      return () => {
        this.rmfIngress.sioClient.unsubscribe(rmfSub);
      };
    }).pipe(share({ connector: () => new BehaviorSubject<T | null>(null) }));
  }

  buildingMap(): Observable<RmfModels.BuildingMap | null> {
    return this._buildingMap;
  }

  doorStates(doorName: string): Observable<RmfModels.DoorState | null> {
    return this._getOrCreate(this._doorStates, doorName, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeDoorState(doorName, cb)),
    );
  }

  doorHealth(doorName: string): Observable<RmfModels.DoorHealth | null> {
    return this._getOrCreate(this._doorHealth, doorName, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeDoorHealth(doorName, cb)),
    );
  }

  liftStates(liftName: string): Observable<RmfModels.LiftState | null> {
    return this._getOrCreate(this._liftStates, liftName, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeLiftState(liftName, cb)),
    );
  }

  liftHealth(liftName: string): Observable<RmfModels.LiftHealth | null> {
    return this._getOrCreate(this._liftHealth, liftName, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeLiftHealth(liftName, cb)),
    );
  }

  dispenserStates(guid: string): Observable<RmfModels.DispenserState | null> {
    return this._getOrCreate(this._dispenserStates, guid, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeDispenserState(guid, cb)),
    );
  }

  dispenserHealth(guid: string): Observable<RmfModels.DispenserHealth | null> {
    return this._getOrCreate(this._dispenserHealth, guid, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeDispenserHealth(guid, cb)),
    );
  }

  ingestorStates(guid: string): Observable<RmfModels.IngestorState | null> {
    return this._getOrCreate(this._ingestorStates, guid, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeIngestorState(guid, cb)),
    );
  }

  ingestorHealth(guid: string): Observable<RmfModels.IngestorHealth | null> {
    return this._getOrCreate(this._ingestorHealth, guid, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeIngestorHealth(guid, cb)),
    );
  }

  fleetStates(fleet: string): Observable<RmfModels.FleetState | null> {
    return this._getOrCreate(this._fleetStates, fleet, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeFleetState(fleet, cb)),
    );
  }

  robotStates(
    fleet: string,
    robot: string,
  ): Observable<{ fleet: string; robotState: RmfModels.RobotState } | null> {
    return this._getOrCreate(this._robotStates, `${fleet}/${robot}`, () =>
      this._fleetStates[fleet].pipe(
        map((fleetState) => {
          if (fleetState === null) {
            return null;
          }
          const robotState = fleetState.robots.find((r) => r.name === robot);
          if (robotState === undefined) {
            return null;
          }
          return { fleet, robotState };
        }),
      ),
    );
  }

  robotHealth(fleet: string, robot: string): Observable<RmfModels.RobotHealth | null> {
    return this._getOrCreate(this._robotHealth, `${fleet}/${robot}`, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeRobotHealth(fleet, robot, cb)),
    );
  }

  taskSummaries(taskId: string): Observable<RmfModels.TaskSummary | null> {
    return this._getOrCreate(this._taskSummaries, taskId, () =>
      this._wrapSioEvent((cb) => this.rmfIngress.sioClient.subscribeTaskSummary(taskId, cb)),
    );
  }
}
