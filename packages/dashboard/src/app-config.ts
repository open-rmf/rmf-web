/* istanbul ignore file */

import { Authenticator, KeycloakAuthenticator, StubAuthenticator } from 'rmf-auth';
import ResourceManager from './managers/resource-manager';
import { BasePath } from './util/url';

export interface AppConfig {
  authenticator: Authenticator;
  trajServerUrl: string;
  rmfServerUrl: string;
  appResourcesFactory: () => Promise<ResourceManager | undefined>;
}

declare global {
  interface Window {
    ENV: any;
  }
}

export const appConfig: AppConfig = (() => {
  const trajServer = window.ENV.REACT_APP_TRAJECTORY_SERVER;
  if (!trajServer) {
    throw new Error('REACT_APP_TRAJECTORY_SERVER env variable is needed but not defined');
  }

  const authenticator = (() => {
    if (!window.ENV.REACT_APP_AUTH_PROVIDER) {
      return new StubAuthenticator();
    }
    // it is important that we do not do any processing on REACT_APP_AUTH_PROVIDER so that webpack
    // can remove dead code, we DO NOT want the output to have the stub authenticator even if
    // it is not used.
    const provider = window.ENV.REACT_APP_AUTH_PROVIDER;
    switch (provider) {
      case 'keycloak':
        if (!window.ENV.REACT_APP_KEYCLOAK_CONFIG) {
          throw new Error('missing REACT_APP_KEYCLOAK_CONFIG');
        }
        return new KeycloakAuthenticator(
          JSON.parse(window.ENV.REACT_APP_KEYCLOAK_CONFIG),
          `${window.location.origin}${BasePath}/silent-check-sso.html`,
        );
      case 'stub':
        return new StubAuthenticator();
      default:
        throw new Error(`unknown auth provider "${provider}"`);
    }
  })();

  if (!window.ENV.REACT_APP_RMF_SERVER) {
    throw new Error('REACT_APP_RMF_SERVER is required');
  }

  return {
    authenticator,
    appResourcesFactory: ResourceManager.defaultResourceManager,
    trajServerUrl: trajServer,
    rmfServerUrl: window.ENV.REACT_APP_RMF_SERVER,
  } as AppConfig;
})();

export default appConfig;
