import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { KeycloakConfig } from 'keycloak-js';
import ApiClient from './api-client';
import Authenticator, { DefaultAuthenticator } from './components/auth/authenticator';
import FakeAuthenticator from './mock/fake-authenticator';
import FakeTrajectoryManager from './mock/fake-traj-manager';
import { FakeTransport } from './mock/fake-transport';
import ResourceManager, { ResourceConfigurationsType } from './resource-manager';
import { DefaultTrajectoryManager, RobotTrajectoryManager } from './robot-trajectory-manager';
import Ros2Transport from './ros2-transport';

export interface AppConfig {
  authenticator: Authenticator;
  appResources: Promise<ResourceConfigurationsType>;
  transportFactory: () => Promise<RomiCore.Transport>;
  trajectoryManagerFactory?: () => Promise<RobotTrajectoryManager>;
  trajServerUrl: string;
}

export const appConfig: AppConfig = (() => {
  if (!process.env.REACT_APP_MOCK && process.env.NODE_ENV !== 'test') {
    const trajServer = process.env.REACT_APP_TRAJECTORY_SERVER;
    if (!trajServer) {
      throw new Error('REACT_APP_TRAJECTORY_SERVER env variable is needed but not defined');
    }

    const authConfig: KeycloakConfig = (() => {
      if (process.env.REACT_APP_AUTH_CONFIG) {
        return JSON.parse(process.env.REACT_APP_AUTH_CONFIG) as KeycloakConfig;
      }
      return {
        realm: 'master',
        clientId: 'romi-dashboard',
        url: 'http://localhost:8080/auth',
      };
    })();
    const authenticator = new DefaultAuthenticator(authConfig);

    if (!process.env.REACT_APP_API_SERVER) {
      throw new Error('REACT_APP_API_SERVER env variable is needed but not defined');
    }
    const apiServer = process.env.REACT_APP_API_SERVER;
    const apiClientPromise = ApiClient.connect(apiServer);

    return {
      authenticator,
      appResources: ResourceManager.getResourceConfigurationFile(),
      trajectoryManagerFactory: () => DefaultTrajectoryManager.create(trajServer),
      transportFactory: async () => {
        const apiClient = await apiClientPromise;
        return new Ros2Transport(apiClient);
      },
      trajServerUrl: trajServer,
    };
  } else {
    return {
      authenticator: new FakeAuthenticator(),
      appResources: ResourceManager.getResourceConfigurationFile(),
      transportFactory: async () => new FakeTransport(),
      trajectoryManagerFactory: async () => new FakeTrajectoryManager(),
      trajServerUrl: '',
    };
  }
})();

export default appConfig;
