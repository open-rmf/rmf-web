import { RobotResourceManager } from './resource-manager-robots';

interface ResourceDispenserConfigurationInterface {
  icons: Record<string, string>;
}

export type ResourceDispenserConfigurationType = Record<
  string,
  ResourceDispenserConfigurationInterface
>;

export class DispenserResourceManager {
  dispensers: ResourceDispenserConfigurationType;
  robots: RobotResourceManager;

  constructor(
    dispenserResources: ResourceDispenserConfigurationType,
    robotsInstance: RobotResourceManager,
  ) {
    this.dispensers = dispenserResources;
    this.robots = robotsInstance;
  }

  getAvailableDispenserPerPlace = (fleetName: string, placeName: string) => {
    return this.robots.getDispensersPerFleet(fleetName, placeName);
  };

  getIconPath = (dispenserName: string): string | null => {
    if (!this.dispenserExists(dispenserName)) {
      return null;
    }
    if (!this.dispensers[dispenserName].hasOwnProperty('icons')) {
      return null;
    }
    const rootIconPath = '/assets/icons';
    const robotIcons = this.dispensers[dispenserName].icons[dispenserName];
    return `${rootIconPath}${robotIcons}`;
  };

  dispenserExists = (dispenserName: string) => {
    if (!this.dispensers.hasOwnProperty(dispenserName)) {
      return false;
    }
    return true;
  };
}
