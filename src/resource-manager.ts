import axios from 'axios';
import { RobotResourceManager, RobotResource } from './resource-manager-robots';
import { DispenserResourceManager, DispenserResource } from './resource-manager-dispensers';

export interface ResourceConfigurationsType {
  robots: Record<string, RobotResource>; // Record<FleetName, RobotResource>
  dispensers?: Record<string, DispenserResource>; // Record<DispenserName, DispenserResource>
}

export default class ResourceManager {
  robots: RobotResourceManager;
  dispensers: DispenserResourceManager | undefined;

  constructor(resources: ResourceConfigurationsType) {
    this.robots = new RobotResourceManager(resources.robots);
    if (resources.dispensers) {
      this.dispensers = new DispenserResourceManager(resources.dispensers);
    }
  }

  static getResourceConfigurationFile = async (): Promise<Partial<ResourceConfigurationsType>> => {
    try {
      // Gets data served by the project itself
      const response = await axios.get('/assets/icons/main.json');
      return response.data as Partial<ResourceConfigurationsType>;
    } catch (error) {
      console.error(error);
      return {};
    }
  };
}

export const resourceManagerFactory = (
  resources: Partial<ResourceConfigurationsType> | undefined,
): ResourceManager => {
  if (!resources) {
    return {} as ResourceManager;
  }

  if (resources.robots && !Object.keys(resources.robots).length) {
    return {} as ResourceManager;
  }

  if (resources?.dispensers && !Object.keys(resources.dispensers).length) {
    const data = Object.assign({}, resources);
    delete data['dispensers'];
    return new ResourceManager(data as ResourceConfigurationsType);
  }

  return new ResourceManager(resources as ResourceConfigurationsType);
};
