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
      return null;
    }
    if (!this.robots[fleetName].hasOwnProperty('icons')) {
      return null;
    }
    const robotIcons = this.robots[fleetName].icons;
    // In case the fleet has different models
    let iconPath: string | null = null;

    if (!!robotModel && robotIcons.hasOwnProperty(robotModel)) {
      iconPath = robotIcons[robotModel];
    } else {
      iconPath = !!robotIcons[fleetName] ? `${robotIcons[fleetName]}` : null;
    }

    try {
      return (await import(/* webpackMode: "eager" */ `../assets/resources${iconPath}`)).default;
    } catch {
      return null;
    }
  };

  getDispensersPerFleet = (fleetName: string, placeName: string): string[] | null => {
    if (!this.fleetExists(fleetName) || !this.placesExists(fleetName)) {
      return null;
    }
    if (!this.robots[fleetName].places.hasOwnProperty(placeName)) {
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
