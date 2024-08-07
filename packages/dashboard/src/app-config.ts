import React from 'react';
import { getDefaultTaskDefinition, TaskDefinition } from 'react-components';

import testConfig from '../app-config.json';
import { Authenticator, KeycloakAuthenticator, StubAuthenticator } from './auth';

export interface RobotResource {
  /**
   * Path to an image to be used as the robot's icon.
   */
  icon?: string;

  /**
   * Scale of the image to match the robot's dimensions.
   */
  scale?: number;
}

export interface FleetResource {
  // TODO(koonpeng): configure robot resources based on robot model, this will require https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/robot_state.json to expose the robot model.
  // [robotModel: string]: RobotResource;
  default: RobotResource;
}

export interface LogoResource {
  /**
   * Path to an image to be used as the logo on the app bar.
   */
  header: string;
}

export interface Resources {
  fleets: { [fleetName: string]: FleetResource };
  logos: LogoResource;
}

/**
 * Configuration for task definitions.
 */
export interface TaskResource {
  /**
   * The task definition to configure.
   */
  taskDefinitionId: 'patrol' | 'delivery' | 'compose-clean' | 'custom_compose';

  /**
   * Configure the display name for the task definition.
   */
  displayName?: string;
}

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
  allowedTasks: TaskResource[];

  /**
   * Set various resources (icons, logo etc) used. Different resource can be used based on the theme, `default` is always required.
   */
  resources: { [theme: string]: Resources; default: Resources };

  // FIXME(koonpeng): this is used for very specific tasks, should be removed when mission
  // system is implemented.
  cartIds: string[];
}

/**
 * These will be injected at build time, they CANNOT be changed after the bundle is built.
 */
export interface BuildConfig {
  baseUrl: string;
  authProvider: 'keycloak' | 'stub';
  customTabs?: boolean;
  adminTab?: boolean;
}

export interface AppConfig extends RuntimeConfig {
  buildConfig: BuildConfig;
}

declare const APP_CONFIG: AppConfig;

const appConfig: AppConfig = (() => {
  if (import.meta.env.PROD) {
    return APP_CONFIG;
  } else {
    // globals cannot be injected in tests so we need a fallback, this should be
    // removed by terser in prod builds.
    return testConfig as AppConfig;
  }
})();

export const AppConfigContext = React.createContext(appConfig);

const authenticator: Authenticator = (() => {
  // must use if statement instead of switch for vite tree shaking to work
  if (APP_CONFIG_AUTH_PROVIDER === 'keycloak') {
    return new KeycloakAuthenticator(
      APP_CONFIG.authConfig as KeycloakAuthConfig,
      `${import.meta.env.BASE_URL}silent-check-sso.html`,
    );
  } else if (APP_CONFIG_AUTH_PROVIDER === 'stub') {
    return new StubAuthenticator();
  } else {
    throw new Error('unknown auth provider');
  }
})();

export const AuthenticatorContext = React.createContext(authenticator);

export const ResourcesContext = React.createContext<Resources>(appConfig.resources.default);

// FIXME(koonepng): This should be fully definition in app config when the dashboard actually
// supports configurating all the fields.
export const allowedTasks: TaskDefinition[] = appConfig.allowedTasks.map((taskResource) => {
  const defaultTaskDefinition = getDefaultTaskDefinition(taskResource.taskDefinitionId);
  if (!defaultTaskDefinition) {
    throw Error(`Invalid tasks configured for dashboard: [${taskResource.taskDefinitionId}]`);
  }
  if (taskResource.displayName !== undefined) {
    return {
      ...defaultTaskDefinition,
      taskDisplayName: taskResource.displayName,
    };
  } else {
    return defaultTaskDefinition;
  }
});
