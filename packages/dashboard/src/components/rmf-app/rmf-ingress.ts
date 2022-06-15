import {
  AdminApi,
  BuildingApi,
  BuildingMap,
  Configuration,
  DefaultApi,
  DispensersApi,
  Door,
  DoorsApi,
  DoorState,
  FleetsApi,
  IngestorsApi,
  Lift,
  LiftsApi,
  LiftState,
  SioClient,
  Subscription as SioSubscription,
  TasksApi,
} from 'api-client';
import axios from 'axios';
import { Authenticator } from 'rmf-auth';
import { map, Observable, share } from 'rxjs';
import appConfig from '../../app-config';
import { NegotiationStatusManager } from '../../managers/negotiation-status-manager';
import {
  DefaultTrajectoryManager,
  RobotTrajectoryManager,
} from '../../managers/robot-trajectory-manager';

export class RmfIngress {
  sioClient: SioClient;
  buildingApi: BuildingApi;
  defaultApi: DefaultApi;
  doorsApi: DoorsApi;
  liftsApi: LiftsApi;
  dispensersApi: DispensersApi;
  ingestorsApi: IngestorsApi;
  fleetsApi: FleetsApi;
  tasksApi: TasksApi;
  adminApi: AdminApi;
  negotiationStatusManager: NegotiationStatusManager;
  trajectoryManager: RobotTrajectoryManager;

  constructor(authenticator: Authenticator) {
    if (!authenticator.user) {
      throw new Error(
        'user is undefined, RmfIngress should only be initialized after the authenticator is ready',
      );
    }

    this.sioClient = (() => {
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
    this.sioClient.sio.on('error', console.error);

    // the axios swagger generator is bugged, it does not properly attach the token so we have
    // to manually add them.
    const axiosInst = axios.create();
    axiosInst.interceptors.request.use(async (req) => {
      await authenticator.refreshToken();
      const token = authenticator.token;
      if (!token) {
        return req;
      }
      req.headers['Authorization'] = `Bearer ${token}`;
      return req;
    });
    const apiConfig = new Configuration({
      accessToken: authenticator.token,
      basePath: appConfig.rmfServerUrl,
    });

    this.buildingApi = new BuildingApi(apiConfig, undefined, axiosInst);
    this.defaultApi = new DefaultApi(apiConfig, undefined, axiosInst);
    this.doorsApi = new DoorsApi(apiConfig, undefined, axiosInst);
    this.liftsApi = new LiftsApi(apiConfig, undefined, axiosInst);
    this.dispensersApi = new DispensersApi(apiConfig, undefined, axiosInst);
    this.ingestorsApi = new IngestorsApi(apiConfig, undefined, axiosInst);
    this.fleetsApi = new FleetsApi(apiConfig, undefined, axiosInst);
    this.tasksApi = new TasksApi(apiConfig, undefined, axiosInst);
    this.adminApi = new AdminApi(apiConfig, undefined, axiosInst);

    const ws = new WebSocket(appConfig.trajServerUrl);
    this.trajectoryManager = new DefaultTrajectoryManager(ws, authenticator);
    this.negotiationStatusManager = new NegotiationStatusManager(ws, authenticator);
  }

  private _convertSioToRxObs<T>(
    key: string,
    cache: Record<string, Observable<T>>,
    sioSubscribe: (key: string, handler: (data: T) => void) => SioSubscription,
  ): Observable<T> {
    if (!cache[key]) {
      cache[key] = new Observable<T>((subscriber) => {
        const sioSub = sioSubscribe(key, subscriber.next.bind(subscriber));
        return () => this.sioClient.unsubscribe(sioSub);
      }).pipe(share());
    }
    return cache[key];
  }

  buildingMapObs: Observable<BuildingMap> = new Observable((subscriber) => {
    this.sioClient.subscribeBuildingMap((data) => subscriber.next(data));
  });

  doorsObs: Observable<Door[]> = this.buildingMapObs.pipe(
    map((buildingMap) => buildingMap.levels.flatMap((level) => level.doors)),
  );

  private _doorStateObsCache: Record<string, Observable<DoorState>> = {};
  getDoorStateObs(name: string): Observable<DoorState> {
    return this._convertSioToRxObs(
      name,
      this._doorStateObsCache,
      this.sioClient.subscribeDoorState.bind(this.sioClient),
    );
  }

  liftsObs: Observable<Lift[]> = this.buildingMapObs.pipe(map((buildingMap) => buildingMap.lifts));

  private _liftStateObsCache: Record<string, Observable<LiftState>> = {};
  getLiftStateObs(name: string): Observable<LiftState> {
    return this._convertSioToRxObs(
      name,
      this._liftStateObsCache,
      this.sioClient.subscribeLiftState.bind(this.sioClient),
    );
  }
}
