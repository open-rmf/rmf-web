import axios from 'axios';

export interface RobotResource {
  icons: Record<string, string>; // Record<ModelName|FleetName, IconPath>
}

export interface ResourceConfigurationsType {
  robots?: Record<string, RobotResource>; // Record<FleetName, RobotResource>
}

export default class ResourceManager {
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
}
