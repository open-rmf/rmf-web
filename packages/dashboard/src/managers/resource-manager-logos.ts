export interface LogoResource {
  icons: Record<string, string>;
}

export class LogoResourceManager {
  logos: Record<string, LogoResource>;

  constructor(logoResources: Record<string, LogoResource>) {
    this.logos = logoResources;
  }

  getIconPath = async (logoName: string): Promise<string | null> => {
    if (!this.logoExists(logoName)) {
      return null;
    }
    if (!this.logos[logoName].hasOwnProperty('icons')) {
      return null;
    }
    const logoIcon = this.logos[logoName].icons[logoName];

    try {
      return (await import(/* webpackMode: "eager" */ `../assets${logoIcon}`)).default;
    } catch {
      return null;
    }
  };

  get all(): Record<string, LogoResource> {
    return this.logos;
  }

  get allValues(): LogoResource[] {
    return Object.values(this.logos);
  }

  private logoExists = (logoName: string) => {
    if (!this.logos.hasOwnProperty(logoName)) {
      return false;
    }
    return true;
  };
}
