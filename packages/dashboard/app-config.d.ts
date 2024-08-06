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
  config: null;
}

export interface KeycloakAuthConfig {
  provider: 'keycloak';
  config: {
    url: string;
    realm: string;
    clientId: string;
  };
}

export interface AppConfigInput {
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
  customTabs?: boolean;
  adminTab?: boolean;
  // FIXME(koonpeng): this is used for very specific tasks, should be removed when mission
  // system is implemented.
  cartIds: string[];
}
