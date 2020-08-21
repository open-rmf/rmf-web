import axios from 'axios';

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
  | string
  | Record<string, string | any>
  | Record<string, Record<string, string | any>>
  | any;
export type ResourceConfigurationsType = Record<string, SubResourceConfigurationsType>;

export class RobotResourceManager {
  resources: ResourceConfigurationsType;

  constructor(resources: ResourceConfigurationsType) {
    this.resources = resources.robots;
  }

  getAvailablePlacesPerFleet = (fleetName: string) => {
    if (!resources.robots || !resources.robots.hasOwnProperty(fleetName)) {
      return null;
    }
    if (!resources.robots[fleetName].hasOwnProperty('places')) {
      return null;
    }
    return resources.robots[fleetName].places;
  };

  static getAvailablePlacesPerFleet = (
    resources: ResourceConfigurationsType,
    fleetName: string,
  ): string[] | null => {
    if (!resources.robots || !resources.robots.hasOwnProperty(fleetName)) {
      return null;
    }
    if (!resources.robots[fleetName].hasOwnProperty('places')) {
      return null;
    }
    return resources.robots[fleetName].places;
  };

  static getRobotIconPath(
    resources: ResourceConfigurationsType,
    fleetName: string,
    robotModel?: string | undefined,
  ): string | null {
    if (!resources.robots || !resources.robots.hasOwnProperty(fleetName)) {
      return null;
    }
    if (!resources.robots[fleetName].hasOwnProperty('icons')) {
      return null;
    }
    const rootIconPath = '/assets/icons';
    const robotIcons = resources.robots[fleetName].icons;
    // In case the fleet has different models
    if (!!robotModel && robotIcons.hasOwnProperty(robotModel)) {
      const iconPath = robotIcons[robotModel];
      return `${rootIconPath}${iconPath}`;
    } else {
      return `${rootIconPath}${robotIcons[fleetName]}`;
    }
  }

  getRobot = () => {
    return 'test';
  };
}

export class DispenserResourceManager {
  resources: ResourceConfigurationsType;
  robots: RobotResourceManager;
  constructor(resources: ResourceConfigurationsType, robotsInstance: RobotResourceManager) {
    this.resources = resources.dispensers;
    this.robots = robotsInstance;
  }
}

export default class ResourceManager {
  robots: RobotResourceManager;
  dispensers: DispenserResourceManager;
  resources: ResourceConfigurationsType;

  constructor(resources: ResourceConfigurationsType) {
    this.robots = new RobotResourceManager(resources);
    this.dispensers = new DispenserResourceManager(resources, this.robots);
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
