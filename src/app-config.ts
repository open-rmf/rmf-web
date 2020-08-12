import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { SossTransport } from '@osrf/romi-js-soss-transport';
import Authenticator from './components/auth/authenticator';
import KeycloakAuthenticator from './components/auth/keycloak-authenticator';
import FakeAuthenticator from './mock/fake-authenticator';
import FakeTrajectoryManager from './mock/fake-traj-manager';
import { FakeTransport } from './mock/fake-transport';
import { DefaultTrajectoryManager, RobotTrajectoryManager } from './robot-trajectory-manager';

export interface AppConfig {
  authenticator: Authenticator;
  transportFactory: () => Promise<RomiCore.Transport>;
  trajectoryManagerFactory: () => Promise<RobotTrajectoryManager>;
}

export default async function makeAppConfig(): Promise<AppConfig> {
  if (!process.env.REACT_APP_MOCK && process.env.NODE_ENV !== 'test') {
    const sossNodeName = process.env.REACT_APP_SOSS_NODE_NAME || 'romi-dashboard';

    const sossServer = process.env.REACT_APP_SOSS_SERVER;
    if (!sossServer) {
      throw new Error('REACT_APP_SOSS_SERVER env variable is needed but not defined');
    }

    const trajServer = process.env.REACT_APP_TRAJECTORY_SERVER;
    if (!trajServer) {
      throw new Error('REACT_APP_TRAJECTORY_SERVER env variable is needed but not defined');
    }

    const authenticator: Authenticator = await (() => {
      if (!process.env.REACT_APP_AUTH_URL) {
        console.warn('using fake authenticator');
        return new FakeAuthenticator();
      }
      const authUrl = process.env.REACT_APP_AUTH_URL || 'http://localhost:8080/auth';
      return KeycloakAuthenticator.create({
        clientId: 'romi-dashboard',
        realm: 'master',
        url: authUrl,
      });
    })();
    if (process.env.REACT_APP_SOSS_TOKEN) {
      console.warn('using hardcoded soss token');
    }
    const sossToken = process.env.REACT_APP_SOSS_TOKEN || authenticator.sossToken || '';

    return {
      authenticator,
      transportFactory: () => SossTransport.connect(sossNodeName, sossServer, sossToken),
      trajectoryManagerFactory: () => DefaultTrajectoryManager.create(trajServer),
    };
  } else {
    return {
      authenticator: new FakeAuthenticator(),
      transportFactory: async () => new FakeTransport(),
      trajectoryManagerFactory: async () => new FakeTrajectoryManager(),
    };
  }
}
