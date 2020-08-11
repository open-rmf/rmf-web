import axios from 'axios';

// We set any here to be more flexible when receiving the configuration structure
type SubResourceConfigurationsType =
  | string
  | Record<string, string | any>
  | Record<string, Record<string, string | any>>
  | any;
export type ResourceConfigurationsType = Record<string, SubResourceConfigurationsType>;
export default class ResourceManager {
  static getResourceConfigurationFile = async (): Promise<ResourceConfigurationsType> => {
    try {
      const response = await axios.get('/assets/icons/main.json');
      return response.data as ResourceConfigurationsType;
    } catch (error) {
      console.error(error);
      return {};
    }
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

    if (!!robotModel && robotIcons.hasOwnProperty(robotModel)) {
      const iconPath = robotIcons[robotModel];
      return `${rootIconPath}${iconPath}`;
    } else {
      return `${rootIconPath}${robotIcons[fleetName]}`;
    }
  }
}
