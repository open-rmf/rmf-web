import Debug from 'debug';

const debug = Debug('ResourceManger');

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
    if (!Object.prototype.hasOwnProperty.call(this.logos[logoName], 'icons')) {
      return null;
    }
    const logoIcon = this.logos[logoName].icons[logoName];
    const currDir = process.cwd();

    try {
      return (await import(/* webpackMode: "eager" */ `${currDir}/src/assets/resources${logoIcon}`))
        .default;
    } catch {
      debug(`failed to load icon for "${logoName}"`);
      return null;
    }
  };

  getHeaderLogoPath = async (): Promise<string> => {
    const currDir = process.cwd();
    const iconPath = await this.getIconPath('headerLogo');
    if (iconPath) return iconPath;
    debug('using default header logo');
    return (await import(/* webpackMode: "eager" */ `${currDir}/src/assets/defaultLogo.png`))
      .default;
  };

  get all(): Record<string, LogoResource> {
    return this.logos;
  }

  get allValues(): LogoResource[] {
    return Object.values(this.logos);
  }

  private logoExists = (logoName: string) => {
    if (!Object.prototype.hasOwnProperty.call(this.logos, logoName)) {
      return false;
    }
    return true;
  };
}
