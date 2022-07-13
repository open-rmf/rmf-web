import Debug from 'debug';
import { ThemeMode } from '../settings';

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
    if (!this.logos[logoName].hasOwnProperty('icons')) {
      return null;
    }
    const logoIcon = this.logos[logoName].icons[logoName];

    try {
      return (await import(/* webpackMode: "eager" */ `../assets/resources${logoIcon}`)).default;
    } catch {
      debug(`failed to load icon for "${logoName}"`);
      return null;
    }
  };

  getHeaderLogoPath = async (theme: ThemeMode): Promise<string> => {
    // FIXME: standardize a way to define different assets for different theme mode.
    const themeIcon =
      theme === ThemeMode.RmfDark
        ? await this.getIconPath('darkThemeLogo')
        : await this.getIconPath('headerLogo');
    if (themeIcon) return themeIcon;
    debug('using default header logo');
    return (await import(/* webpackMode: "eager" */ '../assets/defaultLogo.png')).default;
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
