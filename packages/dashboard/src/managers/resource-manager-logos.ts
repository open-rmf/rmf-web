import { ThemeMode } from '../settings';

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
      return null;
    }
  };

  getHeaderLogoPath = async (theme: ThemeMode): Promise<string> => {
    const iconPath = await this.getIconPath('headerLogo');
    const darkIconPath = await this.getIconPath('darkThemeLogo');
    const themeIcon = theme === ThemeMode.Dark ? iconPath : darkIconPath;
    if (themeIcon) return themeIcon;
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
