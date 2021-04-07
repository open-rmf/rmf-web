import { Configuration, DoorsApi, io, LiftsApi, SioClient, TasksApi } from 'api-client';
import { User } from 'rmf-auth';
import appConfig from '../../app-config';
import { NegotiationStatusManager } from '../../managers/negotiation-status-manager';
import { RobotTrajectoryManager } from '../../managers/robot-trajectory-manager';

export class RmfIngress {
  sioClient?: SioClient;
  doorsApi?: DoorsApi;
  liftsApi?: LiftsApi;
  tasksApi?: TasksApi;
  negotiationStatusManager: NegotiationStatusManager;
  trajectoryManager?: RobotTrajectoryManager;

  constructor(user?: User, trajMgr?: RobotTrajectoryManager) {
    this.negotiationStatusManager = new NegotiationStatusManager(appConfig.trajServerUrl);
    if (!user) {
      return;
    }
    this.sioClient = (() => {
      const token = appConfig.authenticator.token;
      const url = new URL(appConfig.rmfServerUrl);
      const path = url.pathname === '/' ? '' : url.pathname;
      const options: Parameters<typeof io>[1] = {
        path: `${path}/socket.io`,
      };
      if (token) {
        options.auth = { token };
      }
      return io(url.origin, options);
    })();
    this.sioClient.on('error', console.error);

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
    this.tasksApi = new TasksApi(apiConfig);
    this.trajectoryManager = trajMgr;
  }
}
