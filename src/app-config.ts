import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { SossTransport } from '@osrf/romi-js-soss-transport';
import debug from 'debug';
import { KeycloakConfig } from 'keycloak-js';
import Authenticator, { DefaultAuthenticator } from './components/auth/authenticator';
import FakeAuthenticator from './mock/fake-authenticator';
import FakeTrajectoryManager from './mock/fake-traj-manager';
import { FakeTransport } from './mock/fake-transport';
import ResourceManager, { ResourceConfigurationsType } from './resource-manager';
import { DefaultTrajectoryManager, RobotTrajectoryManager } from './robot-trajectory-manager';
import { LOGIN_ROUTE } from './util/url';

export interface AppConfig {
  authenticator: Authenticator;
  authRedirectUri: string;
  appResources: Promise<ResourceConfigurationsType>;
  transportFactory: () => Promise<RomiCore.Transport>;
  trajectoryManagerFactory?: () => Promise<RobotTrajectoryManager>;
}

export const appConfig: AppConfig = (() => {
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

    const redirectUri = getRedirectUri();
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
    const authenticator = new DefaultAuthenticator(authConfig, redirectUri.href);

    return {
      authenticator,
      authRedirectUri: redirectUri.href,
      appResources: ResourceManager.getResourceConfigurationFile(),
      transportFactory: () => {
        const sossToken = process.env.REACT_APP_SOSS_TOKEN || authenticator.sossToken || '';
        debug.log('authenticating to rmf with token', sossToken);
        return SossTransport.connect(sossNodeName, sossServer, sossToken);
      },
      trajectoryManagerFactory: () => DefaultTrajectoryManager.create(trajServer),
    };
  } else {
    return {
      authenticator: new FakeAuthenticator(),
      authRedirectUri: getRedirectUri().href,
      appResources: ResourceManager.getResourceConfigurationFile(),
      transportFactory: async () => new FakeTransport(),
      trajectoryManagerFactory: async () => new FakeTrajectoryManager(),
    };
  }
})();

export default appConfig;

function getRedirectUri(): URL {
  const redirectUri = new URL(window.location.href);
  redirectUri.pathname = LOGIN_ROUTE;
  redirectUri.searchParams.append('response', '1');
  return redirectUri;
}
