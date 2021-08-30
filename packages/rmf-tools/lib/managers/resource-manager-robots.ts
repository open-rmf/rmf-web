import Debug from 'debug';

const debug = Debug('ResourceManager');

export interface RobotResource {
  icons: Record<string, string>; // Record<ModelName|FleetName, IconPath>
  places: Record<string, string[]>; // Record<Places, Dispensers[]>
}

export class RobotResourceManager {
  robots: Record<string, RobotResource>;

  constructor(robotResources: Record<string, RobotResource>) {
    this.robots = robotResources;
  }

  getAvailablePlacesPerFleet = (fleetName: string): string[] | null => {
    if (!this.fleetExists(fleetName) || !this.placesExists(fleetName)) {
      return null;
    }
    return Object.keys(this.robots[fleetName].places);
  };

  getIconPath = async (
    fleetName: string,
    robotModel?: string | undefined,
  ): Promise<string | null> => {
    if (!this.fleetExists(fleetName)) {
      debug(`failed to load icon for "${fleetName}/${robotModel}" (fleet not in resources)`);
      return null;
    }
    if (!Object.prototype.hasOwnProperty.call(this.robots[fleetName], 'icons')) {
      debug(
        `failed to load icon for "${fleetName}/${robotModel}" (fleet/model does not have an icon)`,
      );
      return null;
    }
    const robotIcons = this.robots[fleetName].icons;
    // In case the fleet has different models
    let iconPath: string | null = null;

    if (!!robotModel && Object.prototype.hasOwnProperty.call(robotIcons, robotModel)) {
      iconPath = robotIcons[robotModel];
    } else {
      iconPath = robotIcons[fleetName] ? `${robotIcons[fleetName]}` : null;
    }

    try {
      return (await import(/* webpackMode: "eager" */ `../assets/resources${iconPath}`)).default;
    } catch {
      debug(`failed to load icon for "${fleetName}/${robotModel}" (failed to load icon module)`);
      return null;
    }
  };

  getDispensersPerFleet = (fleetName: string, placeName: string): string[] | null => {
    if (!this.fleetExists(fleetName) || !this.placesExists(fleetName)) {
      debug(
        `failed to load dispensers for "${fleetName}, ${placeName}" (fleet or place does not exist in resources)`,
      );
      return null;
    }
    if (!Object.prototype.hasOwnProperty.call(this.robots[fleetName].places, placeName)) {
      debug(
        `failed to load dispensers for "${fleetName}, ${placeName}" (place does not exist in resources)`,
      );
      return null;
    }
    return this.robots[fleetName].places[placeName];
  };

  private fleetExists = (fleetName: string): boolean => {
    if (!Object.prototype.hasOwnProperty.call(this.robots, fleetName)) {
      return false;
    }
    return true;
  };

  private placesExists = (fleetName: string): boolean => {
    if (!Object.prototype.hasOwnProperty.call(this.robots[fleetName], 'places')) {
      return false;
    }
    return true;
  };
}
