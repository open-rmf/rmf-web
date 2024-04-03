import Debug from 'debug';

const debug = Debug('ResourceManager');

export interface RobotResource {
  icons: Record<string, string>; // Record<ModelName|FleetName, IconPath>
  places: Record<string, string[]>; // Record<Places, Dispensers[]>
  scale: number;
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
    if (!this.robots[fleetName].hasOwnProperty('icons')) {
      debug(
        `failed to load icon for "${fleetName}/${robotModel}" (fleet/model does not have an icon)`,
      );
      return null;
    }
    const robotIcons = this.robots[fleetName].icons;
    // In case the fleet has different models
    let iconPath: string | null = null;

    if (!!robotModel && robotIcons.hasOwnProperty(robotModel)) {
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

  getRobotIconScale = (fleetName: string, robotModel?: string | undefined): number | null => {
    if (!this.fleetExists(fleetName)) {
      debug(`failed to load scale for "${fleetName}/${robotModel}" (fleet not in resources)`);
      return null;
    }

    if (!this.robots[fleetName].hasOwnProperty('scale')) {
      debug(
        `failed to load scale for "${fleetName}/${robotModel}" (fleet/model does not have an scale)`,
      );
      return null;
    }

    const robotScale = this.robots[fleetName].scale;

    if (robotModel) {
      return robotScale;
    }

    return null;
  };

  getDispensersPerFleet = (fleetName: string, placeName: string): string[] | null => {
    if (!this.fleetExists(fleetName) || !this.placesExists(fleetName)) {
      debug(
        `failed to load dispensers for "${fleetName}, ${placeName}" (fleet or place does not exist in resources)`,
      );
      return null;
    }
    if (!this.robots[fleetName].places.hasOwnProperty(placeName)) {
      debug(
        `failed to load dispensers for "${fleetName}, ${placeName}" (place does not exist in resources)`,
      );
      return null;
    }
    return this.robots[fleetName].places[placeName];
  };

  private fleetExists = (fleetName: string): boolean => {
    if (!this.robots.hasOwnProperty(fleetName)) {
      return false;
    }
    return true;
  };

  private placesExists = (fleetName: string): boolean => {
    if (!this.robots[fleetName].hasOwnProperty('places')) {
      return false;
    }
    return true;
  };
}
