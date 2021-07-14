import {
  Configuration,
  DispensersApi,
  DoorsApi,
  FleetsApi,
  IngestorsApi,
  LiftsApi,
  SioClient,
  TasksApi,
} from 'api-client';
import { User } from 'rmf-auth';
import appConfig from '../../app-config';
import { NegotiationStatusManager } from '../../managers/negotiation-status-manager';
import { RobotTrajectoryManager } from '../../managers/robot-trajectory-manager';

export class RmfIngress {
  sioClient: SioClient;
  doorsApi: DoorsApi;
  liftsApi: LiftsApi;
  dispensersApi: DispensersApi;
  ingestorsApi: IngestorsApi;
  fleetsApi: FleetsApi;
  tasksApi: TasksApi;
  negotiationStatusManager: NegotiationStatusManager;
  trajectoryManager?: RobotTrajectoryManager;

  constructor(user: User | null, trajMgr?: RobotTrajectoryManager, ws?: WebSocket) {
    this.negotiationStatusManager = new NegotiationStatusManager(ws, appConfig.authenticator);
    this.sioClient = (() => {
      const token = appConfig.authenticator.token;
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

    const axiosOptions = (() => {
      if (user) {
        return { headers: { Authorization: `Bearer ${user.token}` } };
      }
      return {};
    })();
    const apiConfig: Configuration = {
      accessToken: user?.token || undefined,
      basePath: appConfig.rmfServerUrl,
      baseOptions: axiosOptions,
    };

    this.doorsApi = new DoorsApi(apiConfig);
    this.liftsApi = new LiftsApi(apiConfig);
    this.dispensersApi = new DispensersApi(apiConfig);
    this.ingestorsApi = new IngestorsApi(apiConfig);
    this.fleetsApi = new FleetsApi(apiConfig);
    this.tasksApi = new TasksApi(apiConfig);
    this.trajectoryManager = trajMgr;
  }
}
