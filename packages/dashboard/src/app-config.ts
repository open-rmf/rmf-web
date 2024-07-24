import { Authenticator, KeycloakAuthenticator, StubAuthenticator } from 'rmf-auth';
import { BasePath } from './util/url';

export interface AppConfig {
  authenticator: Authenticator;
  trajServerUrl: string;
  rmfServerUrl: string;
}

export const appConfig: AppConfig = (() => {
  const trajServer = import.meta.env.VITE_TRAJECTORY_SERVER || 'ws://localhost:8006';

  const authenticator = (() => {
    if (!import.meta.env.VITE_AUTH_PROVIDER) {
      return new StubAuthenticator();
    }
    // it is important that we do not do any processing on VITE_AUTH_PROVIDER so that webpack
    // can remove dead code, we DO NOT want the output to have the stub authenticator even if
    // it is not used.
    const provider = import.meta.env.VITE_AUTH_PROVIDER;
    switch (provider) {
      case 'keycloak':
        if (!import.meta.env.VITE_KEYCLOAK_CONFIG) {
          throw new Error('missing VITE_KEYCLOAK_CONFIG');
        }
        return new KeycloakAuthenticator(
          JSON.parse(import.meta.env.VITE_KEYCLOAK_CONFIG),
          `${window.location.origin}${BasePath}/silent-check-sso.html`,
        );
      case 'stub':
        return new StubAuthenticator();
      default:
        throw new Error(`unknown auth provider "${provider}"`);
    }
  })();

  const rmfServerUrl = import.meta.env.VITE_RMF_SERVER || 'http://localhost:8000';

  return {
    authenticator,
    trajServerUrl: trajServer,
    rmfServerUrl,
  } as AppConfig;
})();

export default appConfig;
