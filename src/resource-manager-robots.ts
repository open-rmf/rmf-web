interface ResourceRobotConfigurationInterface {
  icons: Record<string, string>;
  places: Record<string, string[]>;
}

export type ResourceRobotConfigurationType = Record<string, ResourceRobotConfigurationInterface>;

export class RobotResourceManager {
  robots: ResourceRobotConfigurationType;

  constructor(robotResources: ResourceRobotConfigurationType) {
    this.robots = robotResources;
  }

  getAvailablePlacesPerFleet = (fleetName: string) => {
    if (!this.fleetExists(fleetName) || !this.placesExists(fleetName)) {
      return null;
    }
    return this.robots[fleetName].places;
  };

  getRobotIconPath = (fleetName: string, robotModel?: string | undefined): string | null => {
    if (!this.fleetExists(fleetName)) {
      return null;
    }
    if (!this.robots[fleetName].hasOwnProperty('icons')) {
      return null;
    }
    const rootIconPath = '/assets/icons';
    const robotIcons = this.robots[fleetName].icons;
    // In case the fleet has different models
    if (!!robotModel && robotIcons.hasOwnProperty(robotModel)) {
      const iconPath = robotIcons[robotModel];
      return `${rootIconPath}${iconPath}`;
    } else {
      return `${rootIconPath}${robotIcons[fleetName]}`;
    }
  };

  fleetExists = (fleetName: string) => {
    if (!this.robots.hasOwnProperty(fleetName)) {
      return false;
    }
    return true;
  };

  placesExists = (fleetName: string) => {
    if (!this.robots[fleetName].hasOwnProperty('places')) {
      return false;
    }
    return true;
  };

  getDispensersPerFleet = (fleetName: string, placeName: string) => {
    if (!this.fleetExists(fleetName) || !this.placesExists(fleetName)) {
      return null;
    }
    if (!this.robots[fleetName].places.hasOwnProperty(placeName)) {
      return null;
    }
    return this.robots[fleetName].places[placeName];
  };
}
