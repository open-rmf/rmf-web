import testConfig from '../app-config.json';
import { AllowedTask } from './components';
import { Resources } from './hooks/use-resources';
import { Authenticator } from './services/authenticator';
import { KeycloakAuthenticator } from './services/keycloak';
import { StubAuthenticator } from './services/stub-authenticator';

export interface StubAuthConfig {}

export interface KeycloakAuthConfig {
  url: string;
  realm: string;
  clientId: string;
}

/**
 * These config are exposed as a global variable. They can be changed after the bundle is built.
 * To do so, use a placeholder value like `__RMF_SERVER_URL__` and do a search and replace on
 * `index.html` before serving it.
 */
export interface RuntimeConfig {
  /**
   * Url of the RMF api server.
   */
  rmfServerUrl: string;

  /**
   * Url of the RMF trajectory server.
   */
  trajectoryServerUrl: string;

  /**
   * Config for the authentication provider.
   */
  authConfig: KeycloakAuthConfig | StubAuthConfig;

  /**
   * Url to be linked for the "help" button.
   */
  helpLink: string;

  /**
   * Url to be linked for the "report issue" button.
   */
  reportIssue: string;

  /**
   * List of available pickup zones used for delivery tasks.
   */
  pickupZones: string[]; // FIXME(koonpeng): Should be part of task definition

  /**
   * The default zoom level when the map is initially loaded.
   */
  defaultZoom: number;

  /**
   * The default zoom level when a robot is focused on the map.
   */
  defaultRobotZoom: number;

  /**
   * Branding to be shown on the corner of the map.
   */
  attributionPrefix: string;

  /**
   * The default level to be selected when the map is initially loaded.
   */
  defaultMapLevel: string;

  /**
   * List of allowed tasks that can be requested
   */
  allowedTasks: AllowedTask[];

  /**
   * Url to a file to be played when an alert occurs on the dashboard.
   */
  alertAudioPath?: string;

  /**
   * Set various resources (icons, logo etc) used. Different resource can be used based on the theme, `default` is always required.
   */
  resources: { [theme: string]: Resources; default: Resources };

  cartIds: string[];
}

/**
 * These will be injected at build time, they CANNOT be changed after the bundle is built.
 */
export interface BuildConfig {
  /**
   * The base url that the app is served from, this MUST end with a slash.
   */
  baseUrl: string;

  authProvider: 'keycloak' | 'stub';

  /**
   * Whether custom tabs should be enabled, defaults to false.
   */
  customTabs?: boolean;

  /**
   * Whether the admin tab should be enabled, defaults to false.
   */
  adminTab?: boolean;
}

export interface AppConfig extends RuntimeConfig {
  buildConfig: BuildConfig;
}

declare const APP_CONFIG: AppConfig;

export const appConfig: AppConfig = (() => {
  if (import.meta.env.PROD) {
    return APP_CONFIG;
  } else {
    // globals cannot be injected in tests so we need a fallback, this should be
    // removed by terser in prod builds.
    return testConfig as AppConfig;
  }
})();

export const authenticator: Authenticator = (() => {
  // must use if statement instead of switch for vite tree shaking to work
  if (APP_CONFIG_AUTH_PROVIDER === 'keycloak') {
    return new KeycloakAuthenticator(
      APP_CONFIG.authConfig as KeycloakAuthConfig,
      `${location.origin}${import.meta.env.BASE_URL}silent-check-sso.html`,
    );
  } else if (APP_CONFIG_AUTH_PROVIDER === 'stub') {
    return new StubAuthenticator();
  } else {
    throw new Error('unknown auth provider');
  }
})();
