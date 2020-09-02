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
    if (resources.dispensers && this.robots) {
      this.dispensers = new DispenserResourceManager(resources.dispensers, this.robots);
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
