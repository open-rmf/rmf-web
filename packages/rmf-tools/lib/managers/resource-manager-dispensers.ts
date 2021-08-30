import Debug from 'debug';

const debug = Debug('ResourceManager');

interface Location {
  x: number;
  y: number;
  yaw: number;
  level_name: string;
}

export interface RawDispenserResource {
  icons: Record<string, string>;
  location: Location;
}

export interface DispenserResource extends RawDispenserResource {
  guid: string;
}

export class DispenserResourceManager {
  dispensers: Record<string, DispenserResource>;

  constructor(dispenserResources: Record<string, RawDispenserResource>) {
    this.dispensers = this.assignGuidToDispensers(dispenserResources);
  }

  getIconPath = async (dispenserName: string): Promise<string | null> => {
    if (!this.dispenserExists(dispenserName)) {
      debug(`failed to load icon for "${dispenserName}" (dispenser does not exist in resources)`);
      return null;
    }
    if (!Object.prototype.hasOwnProperty.call(this.dispensers[dispenserName], 'icons')) {
      debug(`failed to load icon for "${dispenserName}" (dispenser does not have an icon)`);
      return null;
    }
    const dispenserIcon = this.dispensers[dispenserName].icons[dispenserName];

    const currDir = process.cwd();

    try {
      return (
        await import(/* webpackMode: "eager" */ `${currDir}/src/assets/resources${dispenserIcon}`)
      ).default;
    } catch {
      debug(`failed to load icon for "${dispenserName}" (failed to load icon module)`);
      return null;
    }
  };

  get all(): Record<string, DispenserResource> {
    return this.dispensers;
  }

  get allValues(): DispenserResource[] {
    return Object.values(this.dispensers);
  }

  private assignGuidToDispensers(
    dispensers: Record<string, RawDispenserResource>,
  ): Record<string, DispenserResource> {
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const newDict: any = Object.assign({}, dispensers);
    Object.keys(dispensers).forEach((key) => {
      newDict[key].guid = key;
    });
    return newDict as Record<string, DispenserResource>;
  }

  private dispenserExists = (dispenserName: string) => {
    if (!Object.prototype.hasOwnProperty.call(this.dispensers, dispenserName)) {
      return false;
    }
    return true;
  };
}
