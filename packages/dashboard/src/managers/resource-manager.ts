import { DispenserResourceManager, RawDispenserResource } from './resource-manager-dispensers';
import { LogoResource, LogoResourceManager } from './resource-manager-logos';
import { RobotResource, RobotResourceManager } from './resource-manager-robots';

const ResourceFile = 'resources/main.json';

export interface ResourceConfigurationsType {
  robots?: Record<string, RobotResource>; // Record<FleetName, RobotResource>
  dispensers?: Record<string, RawDispenserResource>; // Record<DispenserName, DispenserResource>
  logos?: Record<string, LogoResource>;
}

interface ResourceManagersProps extends ResourceConfigurationsType {
  robots?: Record<string, RobotResource>;
}

export default class ResourceManager {
  robots: RobotResourceManager;
  dispensers?: DispenserResourceManager;
  logos?: LogoResourceManager;

  static getResourceConfigurationFile = async (): Promise<ResourceManager | undefined> => {
    try {
      // need to use interpolate string to make webpack resolve import at run time and for
      // typescript to not attempt to typecheck it.
      const resources = (await import(
        /* webpackMode: "eager" */ `../assets/${ResourceFile}`
      )) as ResourceConfigurationsType;
      return ResourceManager.resourceManagerFactory(resources);
    } catch {
      return undefined;
    }
  };

  static resourceManagerFactory = (
    resources: ResourceConfigurationsType | undefined,
  ): ResourceManager => {
    if (!resources) {
      return {} as ResourceManager;
    }

    if (resources.robots && !Object.keys(resources.robots).length) {
      return {} as ResourceManager;
    }

    if (resources.dispensers && !Object.keys(resources.dispensers).length) {
      const data = Object.assign({}, resources);
      delete data['dispensers'];
      return new ResourceManager(data as ResourceManagersProps);
    }

    if (resources.logos && !Object.keys(resources.logos).length) {
      const data = Object.assign({}, resources);
      delete data['logos'];
      return new ResourceManager(data as ResourceManagersProps);
    }

    return new ResourceManager(resources as ResourceManagersProps);
  };

  constructor(resources: ResourceManagersProps) {
    this.robots = new RobotResourceManager(resources.robots || {});
    if (resources.dispensers) {
      this.dispensers = new DispenserResourceManager(resources.dispensers);
    }
    if (resources.logos) {
      this.logos = new LogoResourceManager(resources.logos);
    }
  }
}
