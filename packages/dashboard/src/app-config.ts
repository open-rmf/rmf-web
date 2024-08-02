import React from 'react';
import { getDefaultTaskDefinition, TaskDefinition } from 'react-components';
import { Authenticator, KeycloakAuthenticator, StubAuthenticator } from 'rmf-auth';

import appConfigJson from '../app-config.json';
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

export interface StubAuthConfig {
  provider: 'stub';
}

export interface KeycloakAuthConfig {
  provider: 'keycloak';
  config: {
    url: string;
    realm: string;
    clientId: string;
  };
}

export interface AppConfig {
  auth: KeycloakAuthConfig | StubAuthConfig;
  helpLink: string;
  reportIssue: string;
  pickupZones: string[];
  defaultZoom: number;
  defaultRobotZoom: number;
  attributionPrefix: string;
  defaultMapLevel: string;
  allowedTasks: TaskResource[];
  resources: Record<string, Resources> & Record<'default', Resources>;
  customTabs?: boolean;
  adminTab?: boolean;
  // FIXME(koonpeng): this is used for very specific tasks, should be removed when mission
  // system is implemented.
  cartIds: string[];
}

export interface AppConfigJson extends AppConfig {
  // these will be injected as global variables
  rmfServerUrl: string;
  trajectoryServerUrl: string;
}

// we specifically don't export app config to force consumers to use the context.
const appConfig: AppConfig = appConfigJson as AppConfig;

export const AppConfigContext = React.createContext(appConfig);

const authenticator: Authenticator = (() => {
  // must use if statement instead of switch for vite tree shaking to work
  if (APP_CONFIG_AUTH_PROVIDER === 'keycloak') {
    return new KeycloakAuthenticator(
      (appConfig.auth as KeycloakAuthConfig).config,
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
