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
import { map, Observable, shareReplay } from 'rxjs';

import { AppConfig } from '../../app-config';
import { Authenticator } from '../../auth';
import { NegotiationStatusManager } from '../../managers/negotiation-status-manager';
import {
  DefaultTrajectoryManager,
  RobotTrajectoryManager,
} from '../../managers/robot-trajectory-manager';

export class RmfIngress {
  // This should be private because socketio does not support "replaying" subscription. If
  // subscription is made before the one made by the observables, the replays will not work
  // correctly.
  private _sioClient: SioClient;

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

  constructor(appConfig: AppConfig, authenticator: Authenticator) {
    if (!authenticator.user) {
      throw new Error(
        'user is undefined, RmfIngress should only be initialized after the authenticator is ready',
      );
    }

    this._sioClient = (() => {
      const token = authenticator.token;
      const url = new URL(appConfig.rmfServerUrl);
      const path = url.pathname === '/' ? '' : url.pathname;

      const options: ConstructorParameters<typeof SioClient>[1] = {
        path: `${path}/socket.io`,
      };
      if (token) {
        options.auth = { token };
      }
      return new SioClient(url.origin, options);
    })();
    this._sioClient.sio.on('error', console.error);

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
    axiosInst.interceptors.response.use(
      (response) => response,
      (error) => {
        console.error(`Axios response error: ${error}`);
        if (error.response.status === 401) {
          window.location.href = '/';
        }
      },
    );
    const apiConfig = new Configuration({
      accessToken: authenticator.token,
      basePath: appConfig.rmfServerUrl,
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

    try {
      const ws = new WebSocket(appConfig.trajectoryServerUrl);
      this.trajectoryManager = new DefaultTrajectoryManager(ws, authenticator);
      this.negotiationStatusManager = new NegotiationStatusManager(ws, authenticator);
    } catch (e) {
      const errorMessage = `Failed to connect to trajectory server at [${appConfig.trajectoryServerUrl}], ${
        (e as Error).message
      }`;
      console.error(errorMessage);
      return;
    }
  }

  private _convertSioToRxObs<T>(
    sioSubscribe: (handler: (data: T) => void) => SioSubscription,
  ): Observable<T> {
    return new Observable<T>((subscriber) => {
      let sioSub: SioSubscription | null = null;
      const onConnect = () => {
        sioSub = sioSubscribe(subscriber.next.bind(subscriber));
      };
      onConnect();
      this._sioClient.sio.on('connect', onConnect);
      return () => {
        sioSub && this._sioClient.unsubscribe(sioSub);
        this._sioClient.sio.off('connect', onConnect);
      };
    }).pipe(shareReplay(1));
  }

  buildingMapObs: Observable<BuildingMap> = this._convertSioToRxObs((handler) =>
    this._sioClient.subscribeBuildingMap(handler),
  );

  beaconsObsStore: Observable<BeaconState> = this._convertSioToRxObs((handler) =>
    this._sioClient.subscribeBeaconState(handler),
  );

  doorsObs: Observable<Door[]> = this.buildingMapObs.pipe(
    map((buildingMap) => buildingMap.levels.flatMap((level) => level.doors)),
  );

  private _doorStateObsStore: Record<string, Observable<DoorState>> = {};
  getDoorStateObs(name: string): Observable<DoorState> {
    if (!this._doorStateObsStore[name]) {
      this._doorStateObsStore[name] = this._convertSioToRxObs((handler) =>
        this._sioClient.subscribeDoorState(name, handler),
      );
    }
    return this._doorStateObsStore[name];
  }

  liftsObs: Observable<Lift[]> = this.buildingMapObs.pipe(map((buildingMap) => buildingMap.lifts));

  private _liftStateObsStore: Record<string, Observable<LiftState>> = {};
  getLiftStateObs(name: string): Observable<LiftState> {
    if (!this._liftStateObsStore[name]) {
      this._liftStateObsStore[name] = this._convertSioToRxObs((handler) =>
        this._sioClient.subscribeLiftState(name, handler),
      );
    }
    return this._liftStateObsStore[name];
  }

  dispensersObs: Observable<Dispenser[]> = new Observable<Dispenser[]>((subscriber) => {
    (async () => {
      const dispensers = (await this.dispensersApi.getDispensersDispensersGet()).data;
      subscriber.next(dispensers);
    })();
  }).pipe(shareReplay(1));

  private _dispenserStateObsStore: Record<string, Observable<DispenserState>> = {};
  getDispenserStateObs(guid: string): Observable<DispenserState> {
    if (!this._dispenserStateObsStore[guid]) {
      this._dispenserStateObsStore[guid] = this._convertSioToRxObs((handler) =>
        this._sioClient.subscribeDispenserState(guid, handler),
      );
    }
    return this._dispenserStateObsStore[guid];
  }

  ingestorsObs: Observable<Ingestor[]> = new Observable<Ingestor[]>((subscriber) => {
    (async () => {
      const ingestors = (await this.ingestorsApi.getIngestorsIngestorsGet()).data;
      subscriber.next(ingestors);
    })();
  }).pipe(shareReplay(1));

  private _ingestorStateObsStore: Record<string, Observable<IngestorState>> = {};
  getIngestorStateObs(guid: string): Observable<IngestorState> {
    if (!this._ingestorStateObsStore[guid]) {
      this._ingestorStateObsStore[guid] = this._convertSioToRxObs((handler) =>
        this._sioClient.subscribeIngestorState(guid, handler),
      );
    }
    return this._ingestorStateObsStore[guid];
  }

  fleetsObs: Observable<FleetState[]> = new Observable<FleetState[]>((subscriber) => {
    (async () => {
      const fleets = (await this.fleetsApi.getFleetsFleetsGet()).data;
      subscriber.next(fleets);
    })();
  }).pipe(shareReplay(1));

  private _fleetStateObsStore: Record<string, Observable<FleetState>> = {};
  getFleetStateObs(name: string): Observable<FleetState> {
    if (!this._fleetStateObsStore[name]) {
      this._fleetStateObsStore[name] = this._convertSioToRxObs((handler) =>
        this._sioClient.subscribeFleetState(name, handler),
      );
    }
    return this._fleetStateObsStore[name];
  }

  private _taskStateObsStore: Record<string, Observable<TaskState>> = {};
  getTaskStateObs(taskId: string): Observable<TaskState> {
    if (!this._taskStateObsStore[taskId]) {
      this._taskStateObsStore[taskId] = this._convertSioToRxObs((handler) =>
        this._sioClient.subscribeTaskState(taskId, handler),
      );
    }
    return this._taskStateObsStore[taskId];
  }

  alertRequestsObsStore: Observable<AlertRequest> = this._convertSioToRxObs((handler) =>
    this._sioClient.subscribeAlertRequests(handler),
  );

  alertResponsesObsStore: Observable<AlertResponse> = this._convertSioToRxObs((handler) =>
    this._sioClient.subscribeAlertResponses(handler),
  );

  deliveryAlertObsStore: Observable<DeliveryAlert> = this._convertSioToRxObs((handler) =>
    this._sioClient.subscribeDeliveryAlerts(handler),
  );
}
