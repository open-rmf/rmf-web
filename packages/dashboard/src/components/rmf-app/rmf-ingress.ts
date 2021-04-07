import { Configuration, DoorsApi, io, LiftsApi, SioClient, TasksApi } from 'api-client';
import { User } from 'rmf-auth';
import appConfig from '../../app-config';
import { NegotiationStatusManager } from '../../managers/negotiation-status-manager';
import { RobotTrajectoryManager } from '../../managers/robot-trajectory-manager';

export class RmfIngress {
  sioClient: SioClient;
  doorsApi: DoorsApi;
  liftsApi: LiftsApi;
  tasksApi: TasksApi;
  negotiationStatusManager: NegotiationStatusManager;
  trajectoryManager?: RobotTrajectoryManager;

  constructor(user?: User | null, trajMgr?: RobotTrajectoryManager) {
    this.sioClient = (() => {
      const token = appConfig.authenticator.token;
      const options: Parameters<typeof io>[1] = {};
      if (token) {
        options.auth = { token };
      }
      return io(appConfig.rmfServerUrl, options);
    })();
    this.sioClient.on('error', console.error);

    const apiConfig: Configuration = {
      accessToken: user?.token || undefined,
      basePath: appConfig.rmfServerUrl,
    };

    this.doorsApi = new DoorsApi(apiConfig);
    this.liftsApi = new LiftsApi(apiConfig);
    this.tasksApi = new TasksApi(apiConfig);
    this.negotiationStatusManager = new NegotiationStatusManager(appConfig.trajServerUrl);
    this.trajectoryManager = trajMgr;
  }
}
