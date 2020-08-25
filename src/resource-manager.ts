import axios from 'axios';
import { RobotResourceManager, ResourceRobotConfigurationType } from './resource-manager-robots';
import {
  DispenserResourceManager,
  ResourceDispenserConfigurationType,
} from './resource-manager-dispensers';

// export interface RobotResource {
//   icons: Record<string, string>; // Record<ModelName|FleetName, IconPath>
// }

// export interface ResourceConfigurationsType {
//   robots?: Record<string, RobotResource>; // Record<FleetName, RobotResource>
// }

// export default class ResourceManager {
//   static getResourceConfigurationFile = async (): Promise<ResourceConfigurationsType> => {
//     try {
//       // Gets data served by the project itself
//       const response = await axios.get('/assets/icons/main.json');
//       return response.data as ResourceConfigurationsType;
//     } catch (error) {
//       console.error(error);
//       return {};
// We set any here to be more flexible when receiving the configuration structure
type SubResourceConfigurationsType =
  | ResourceRobotConfigurationType
  | ResourceDispenserConfigurationType
  | string
  | Record<string, string | any>
  | Record<string, Record<string, string | any>>
  | any;

export type ResourceConfigurationsType = Record<
  'robots' | 'dispensers' | string,
  SubResourceConfigurationsType
>;

export default class ResourceManager {
  robots: RobotResourceManager;
  dispensers: DispenserResourceManager;
  resources: ResourceConfigurationsType;

  constructor(resources: ResourceConfigurationsType) {
    this.robots = new RobotResourceManager(resources.robots);
    this.dispensers = new DispenserResourceManager(resources.dispensers, this.robots);
    this.resources = resources;
  }

  static getResourceConfigurationFile = async (): Promise<ResourceConfigurationsType> => {
    try {
      // Gets data served by the project itself
      const response = await axios.get('/assets/icons/main.json');
      return response.data as ResourceConfigurationsType;
    } catch (error) {
      console.error(error);
      return {};
    }
  };
}
