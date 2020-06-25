import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { SossTransport } from '@osrf/romi-js-soss-transport';
import FakeTrajectoryManager from './mock/fake-traj-manager';
import { FakeTransport } from './mock/fake-transport';
import { DefaultTrajectoryManager, RobotTrajectoryManager } from './robot-trajectory-manager';

export interface AppConfig {
  transportFactory: () => Promise<RomiCore.Transport>;
  trajectoryManagerFactory?: () => Promise<RobotTrajectoryManager>;
}

export let appConfig: AppConfig;

if (!process.env.REACT_APP_MOCK && process.env.NODE_ENV !== 'test') {
  const sossNodeName = process.env.REACT_APP_SOSS_NODE_NAME || 'romi-dashboard';

  const sossServer = process.env.REACT_APP_SOSS_SERVER;
  if (!sossServer) {
    throw new Error('REACT_APP_SOSS_SERVER env variable is needed but not defined');
  }

  // TODO: get token from some auth service
  const token = process.env.REACT_APP_SOSS_TOKEN || '';

  const trajServer = process.env.REACT_APP_TRAJECTORY_SERVER;
  if (!trajServer) {
    throw new Error('REACT_APP_TRAJECTORY_SERVER env variable is needed but not defined');
  }

  appConfig = {
    transportFactory: () => SossTransport.connect(sossNodeName, sossServer, token),
    trajectoryManagerFactory: () => DefaultTrajectoryManager.create(trajServer),
  };
} else {
  appConfig = {
    transportFactory: async () => new FakeTransport(),
    trajectoryManagerFactory: async () => new FakeTrajectoryManager(),
  };
}

export default appConfig;
