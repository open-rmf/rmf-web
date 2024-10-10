import {
  AdminApi,
  AlertRequest,
  AlertResponse,
  AlertsApi,
  BeaconsApi,
  BeaconState,
  BuildingApi,
  BuildingMap,
  Configuration,
  DefaultApi,
  DeliveryAlert,
  DeliveryAlertsApi,
  Dispenser,
  DispensersApi,
  DispenserState,
  Door,
  DoorsApi,
  DoorState,
  FleetsApi,
  FleetState,
  Ingestor,
  IngestorsApi,
  IngestorState,
  Lift,
  LiftsApi,
  LiftState,
  SioClient,
  Subscription as SioSubscription,
  TasksApi,
  TaskStateOutput as TaskState,
} from 'api-client';
import axios from 'axios';
import { EMPTY, map, Observable, of, shareReplay, switchAll, switchMap } from 'rxjs';

import { Authenticator } from './authenticator';
import { NegotiationStatusManager } from './negotiation-status-manager';
import { DefaultTrajectoryManager, RobotTrajectoryManager } from './robot-trajectory-manager';

export interface RmfApi {
  beaconsApi: BeaconsApi;
  buildingApi: BuildingApi;
  defaultApi: DefaultApi;
  doorsApi: DoorsApi;
  liftsApi: LiftsApi;
  dispensersApi: DispensersApi;
  ingestorsApi: IngestorsApi;
  fleetsApi: FleetsApi;
  tasksApi: TasksApi;
  alertsApi: AlertsApi;
  adminApi: AdminApi;
  deliveryAlertsApi: DeliveryAlertsApi;
  negotiationStatusManager?: NegotiationStatusManager;
  trajectoryManager?: RobotTrajectoryManager;

  buildingMapObs: Observable<BuildingMap>;
  beaconsObsStore: Observable<BeaconState>;
  doorsObs: Observable<Door[]>;
  getDoorStateObs(name: string): Observable<DoorState>;
  liftsObs: Observable<Lift[]>;
  getLiftStateObs(name: string): Observable<LiftState>;
  dispensersObs: Observable<Dispenser[]>;
  getDispenserStateObs(guid: string): Observable<DispenserState>;
  ingestorsObs: Observable<Ingestor[]>;
  getIngestorStateObs(guid: string): Observable<IngestorState>;
  fleetsObs: Observable<FleetState[]>;
  getFleetStateObs(name: string): Observable<FleetState>;
  getTaskStateObs(taskId: string): Observable<TaskState>;
  alertRequestsObsStore: Observable<AlertRequest>;
  alertResponsesObsStore: Observable<AlertResponse>;
  deliveryAlertObsStore: Observable<DeliveryAlert>;
}

export class DefaultRmfApi implements RmfApi {
  private _sioClient: Observable<SioClient | null>;

  beaconsApi: BeaconsApi;
  buildingApi: BuildingApi;
  defaultApi: DefaultApi;
  doorsApi: DoorsApi;
  liftsApi: LiftsApi;
  dispensersApi: DispensersApi;
  ingestorsApi: IngestorsApi;
  fleetsApi: FleetsApi;
  tasksApi: TasksApi;
  alertsApi: AlertsApi;
  adminApi: AdminApi;
  deliveryAlertsApi: DeliveryAlertsApi;
  negotiationStatusManager?: NegotiationStatusManager;
  trajectoryManager?: RobotTrajectoryManager;

  constructor(
    private apiServerUrl: string,
    trajectoryServerUrl: string,
    private authenticator: Authenticator,
  ) {
    // This ensures that old connections are disconnected. `this._makeSioClient` returns an
    // observable of `SioClient` that disconnects when it is unsubscribed. `switchAll` will
    // unsubscribe and switch to the new observable when the user changes.
    this._sioClient = new Observable<Observable<SioClient | null>>((subscriber) => {
      subscriber.next(this._makeSioClient());
      this.authenticator.on('userChanged', () => subscriber.next(this._makeSioClient()));
    }).pipe(switchAll(), shareReplay(1));

    // the axios swagger generator is bugged, it does not properly attach the token so we have
    // to manually add them.
    const axiosInst = axios.create();
    axiosInst.interceptors.request.use(
      async (req) => {
        await authenticator.refreshToken();
        const token = authenticator.token;
        if (!token) {
          return req;
        }
        req.headers['Authorization'] = `Bearer ${token}`;
        return req;
      },
      (error) => {
        console.error(`Axios request error: ${error}`);
      },
    );
    const apiConfig = new Configuration({
      accessToken: authenticator.token,
      basePath: apiServerUrl,
    });

    this.beaconsApi = new BeaconsApi(apiConfig, undefined, axiosInst);
    this.buildingApi = new BuildingApi(apiConfig, undefined, axiosInst);
    this.defaultApi = new DefaultApi(apiConfig, undefined, axiosInst);
    this.doorsApi = new DoorsApi(apiConfig, undefined, axiosInst);
    this.liftsApi = new LiftsApi(apiConfig, undefined, axiosInst);
    this.dispensersApi = new DispensersApi(apiConfig, undefined, axiosInst);
    this.ingestorsApi = new IngestorsApi(apiConfig, undefined, axiosInst);
    this.fleetsApi = new FleetsApi(apiConfig, undefined, axiosInst);
    this.tasksApi = new TasksApi(apiConfig, undefined, axiosInst);
    this.alertsApi = new AlertsApi(apiConfig, undefined, axiosInst);
    this.adminApi = new AdminApi(apiConfig, undefined, axiosInst);
    this.deliveryAlertsApi = new DeliveryAlertsApi(apiConfig, undefined, axiosInst);

    this.buildingMapObs = this._convertSioToRxObs((sioClient, handler) =>
      sioClient.subscribeBuildingMap(handler),
    );

    this.beaconsObsStore = this._convertSioToRxObs((sioClient, handler) =>
      sioClient.subscribeBeaconState(handler),
    );

    this.doorsObs = this.buildingMapObs.pipe(
      map((buildingMap) => buildingMap.levels.flatMap((level) => level.doors)),
    );

    this.liftsObs = this.buildingMapObs.pipe(map((buildingMap) => buildingMap.lifts));

    this.dispensersObs = new Observable<Dispenser[]>((subscriber) => {
      (async () => {
        const dispensers = (await this.dispensersApi.getDispensersDispensersGet()).data;
        subscriber.next(dispensers);
      })();
    }).pipe(shareReplay(1));

    this.ingestorsObs = new Observable<Ingestor[]>((subscriber) => {
      (async () => {
        const ingestors = (await this.ingestorsApi.getIngestorsIngestorsGet()).data;
        subscriber.next(ingestors);
      })();
    }).pipe(shareReplay(1));

    this.fleetsObs = new Observable<FleetState[]>((subscriber) => {
      (async () => {
        const fleets = (await this.fleetsApi.getFleetsFleetsGet()).data;
        subscriber.next(fleets);
      })();
    }).pipe(shareReplay(1));

    this.alertRequestsObsStore = this._convertSioToRxObs((sioClient, handler) =>
      sioClient.subscribeAlertRequests(handler),
    );

    this.alertResponsesObsStore = this._convertSioToRxObs((sioClient, handler) =>
      sioClient.subscribeAlertResponses(handler),
    );

    this.deliveryAlertObsStore = this._convertSioToRxObs((sioClient, handler) =>
      sioClient.subscribeDeliveryAlerts(handler),
    );

    try {
      const ws = new WebSocket(trajectoryServerUrl);
      this.trajectoryManager = new DefaultTrajectoryManager(ws, authenticator);
      this.negotiationStatusManager = new NegotiationStatusManager(ws, authenticator);
    } catch (e) {
      const errorMessage = `Failed to connect to trajectory server at [${trajectoryServerUrl}], ${
        (e as Error).message
      }`;
      console.error(errorMessage);
      return;
    }
  }

  private _makeSioClient(): Observable<SioClient | null> {
    if (!this.authenticator.user) {
      return of(null);
    }

    return new Observable((subscriber) => {
      const url = new URL(this.apiServerUrl);
      const path = url.pathname === '/' ? '' : url.pathname;
      const options: ConstructorParameters<typeof SioClient>[1] = {
        path: `${path}/socket.io`,
      };
      options.auth = async (cb) => {
        await this.authenticator.refreshToken();
        if (this.authenticator.token) {
          cb({ token: this.authenticator.token });
        } else {
          cb({});
        }
      };
      const sioClient = new SioClient(url.origin, options);
      sioClient.sio.on('error', console.error);
      subscriber.next(sioClient);
      return () => sioClient.sio.disconnect();
    });
  }

  private _convertSioToRxObs<T>(
    sioSubscribe: (sioClient: SioClient, handler: (data: T) => void) => SioSubscription,
  ): Observable<T> {
    return this._sioClient.pipe(
      switchMap((sioClient) => {
        if (!sioClient) {
          return EMPTY;
        }
        return new Observable<T>((subscriber) => {
          let sioSub: SioSubscription | null = null;
          const onConnect = () => {
            sioSub = sioSubscribe(sioClient, subscriber.next.bind(subscriber));
          };
          onConnect();
          sioClient.sio.on('connect', onConnect);
          return () => {
            sioSub && sioClient.unsubscribe(sioSub);
            sioClient.sio.off('connect', onConnect);
          };
        });
      }),
      shareReplay(1),
    );
  }

  buildingMapObs: Observable<BuildingMap>;
  beaconsObsStore: Observable<BeaconState>;
  doorsObs: Observable<Door[]>;

  private _doorStateObsStore: Record<string, Observable<DoorState>> = {};
  getDoorStateObs(name: string): Observable<DoorState> {
    if (!this._doorStateObsStore[name]) {
      this._doorStateObsStore[name] = this._convertSioToRxObs((sioClient, handler) =>
        sioClient.subscribeDoorState(name, handler),
      );
    }
    return this._doorStateObsStore[name];
  }

  liftsObs: Observable<Lift[]>;
  private _liftStateObsStore: Record<string, Observable<LiftState>> = {};
  getLiftStateObs(name: string): Observable<LiftState> {
    if (!this._liftStateObsStore[name]) {
      this._liftStateObsStore[name] = this._convertSioToRxObs((sioClient, handler) =>
        sioClient.subscribeLiftState(name, handler),
      );
    }
    return this._liftStateObsStore[name];
  }

  dispensersObs: Observable<Dispenser[]>;
  private _dispenserStateObsStore: Record<string, Observable<DispenserState>> = {};
  getDispenserStateObs(guid: string): Observable<DispenserState> {
    if (!this._dispenserStateObsStore[guid]) {
      this._dispenserStateObsStore[guid] = this._convertSioToRxObs((sioClient, handler) =>
        sioClient.subscribeDispenserState(guid, handler),
      );
    }
    return this._dispenserStateObsStore[guid];
  }

  ingestorsObs: Observable<Ingestor[]>;
  private _ingestorStateObsStore: Record<string, Observable<IngestorState>> = {};
  getIngestorStateObs(guid: string): Observable<IngestorState> {
    if (!this._ingestorStateObsStore[guid]) {
      this._ingestorStateObsStore[guid] = this._convertSioToRxObs((sioClient, handler) =>
        sioClient.subscribeIngestorState(guid, handler),
      );
    }
    return this._ingestorStateObsStore[guid];
  }

  // NOTE: This only emits once and doesn't update when the fleet changes.
  fleetsObs: Observable<FleetState[]>;
  private _fleetStateObsStore: Record<string, Observable<FleetState>> = {};
  getFleetStateObs(name: string): Observable<FleetState> {
    if (!this._fleetStateObsStore[name]) {
      this._fleetStateObsStore[name] = this._convertSioToRxObs((sioClient, handler) =>
        sioClient.subscribeFleetState(name, handler),
      );
    }
    return this._fleetStateObsStore[name];
  }

  private _taskStateObsStore: Record<string, Observable<TaskState>> = {};
  getTaskStateObs(taskId: string): Observable<TaskState> {
    if (!this._taskStateObsStore[taskId]) {
      this._taskStateObsStore[taskId] = this._convertSioToRxObs((sioClient, handler) =>
        sioClient.subscribeTaskState(taskId, handler),
      );
    }
    return this._taskStateObsStore[taskId];
  }

  alertRequestsObsStore: Observable<AlertRequest>;
  alertResponsesObsStore: Observable<AlertResponse>;
  deliveryAlertObsStore: Observable<DeliveryAlert>;
}
