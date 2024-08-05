import React from 'react';
import { getDefaultTaskDefinition, TaskDefinition } from 'react-components';

import testConfig from '../app-config.json';
import { Authenticator, KeycloakAuthenticator, StubAuthenticator } from './auth';
import { BasePath } from './util/url';

export interface RobotResource {
  icon?: string;
  scale?: number;
}

export interface FleetResource {
  default: RobotResource;
}

export interface LogoResource {
  header: string;
}

export interface Resources {
  fleets: Record<string, FleetResource>;
  logos: LogoResource;
}

export interface TaskResource {
  taskDefinitionId: string;
  displayName?: string;
}

export interface StubAuthConfig {}

export interface KeycloakAuthConfig {
  url: string;
  realm: string;
  clientId: string;
}

export interface RuntimeConfig {
  rmfServerUrl: string;
  trajectoryServerUrl: string;
  authConfig: KeycloakAuthConfig | StubAuthConfig;
  helpLink: string;
  reportIssue: string;
  pickupZones: string[];
  defaultZoom: number;
  defaultRobotZoom: number;
  attributionPrefix: string;
  defaultMapLevel: string;
  allowedTasks: TaskResource[];
  resources: Record<string, Resources> & Record<'default', Resources>;
  // FIXME(koonpeng): this is used for very specific tasks, should be removed when mission
  // system is implemented.
  cartIds: string[];
}

// these will be injected as defines and potentially be tree shaken out
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
      `${window.location.origin}${BasePath}/silent-check-sso.html`,
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
