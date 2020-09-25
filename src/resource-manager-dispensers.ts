interface Location {
  x: number;
  y: number;
  yaw: number;
  level_name: string;
}

export interface DispenserResource {
  icons: Record<string, string>;
  location: Location;
  guid?: string;
}

export class DispenserResourceManager {
  dispensers: Record<string, DispenserResource>;

  constructor(dispenserResources: Record<string, DispenserResource>) {
    this.dispensers = dispenserResources;
  }

  getIconPath = (dispenserName: string): string | null => {
    if (!this.dispenserExists(dispenserName)) {
      return null;
    }
    if (!this.dispensers[dispenserName].hasOwnProperty('icons')) {
      return null;
    }
    const rootIconPath = '/assets/icons';
    const dispenserIcon = this.dispensers[dispenserName].icons[dispenserName];

    return dispenserIcon ? `${rootIconPath}${dispenserIcon}` : null;
  };

  get all(): Record<string, DispenserResource> {
    return this.dispensers;
  }

  get allValues(): Required<DispenserResource>[] {
    let newDict = Object.assign({}, this.dispensers);
    Object.keys(this.dispensers).forEach((key) => {
      newDict[key].guid = key;
    });
    return Object.values(newDict as Record<string, Required<DispenserResource>>);
  }

  private dispenserExists = (dispenserName: string) => {
    if (!this.dispensers.hasOwnProperty(dispenserName)) {
      return false;
    }
    return true;
  };
}
