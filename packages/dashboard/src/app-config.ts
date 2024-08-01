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

export interface AuthConfig {
  provider: string;
}

export interface AppConfig {
  rmfServerUrl: string;
  trajectoryServerUrl: string;
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
  customTabs: boolean;
  adminTab: boolean;
  // FIXME(koonpeng): this is used for very specific tasks, should be removed when mission
  // system is implemented.
  cartIds: string[];
}

// we specifically don't export app config to force consumers to use the context.
const appConfig: AppConfig = appConfigJson as AppConfig;

export const AppConfigContext = React.createContext(appConfig);

const authenticator: Authenticator = (() => {
  switch (appConfig.auth.provider) {
    case 'keycloak':
      if (!import.meta.env.VITE_KEYCLOAK_CONFIG) {
        throw new Error('missing VITE_KEYCLOAK_CONFIG');
      }
      return new KeycloakAuthenticator(
        appConfig.auth.config,
        `${window.location.origin}${BasePath}/silent-check-sso.html`,
      );
    case 'stub':
      return new StubAuthenticator();
    default:
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
