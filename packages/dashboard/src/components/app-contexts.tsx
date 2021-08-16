import React from 'react';
import appConfig, { AppConfig } from '../app-config';
import ResourceManager from '../managers/resource-manager';
import { defaultSettings } from '../settings';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager | undefined>(undefined);

export const SettingsContext = React.createContext(defaultSettings());

export interface AppController {
  setShowSettings: React.Dispatch<React.SetStateAction<boolean>>;
  showHelp(show: boolean): void;
  toggleHelp(): void;
  showHotkeysDialog(show: boolean): void;
  toggleHotkeysDialog(): void;
  /**
   * FIXME: Move this to settings, this sets if we should show the tooltip icon, and not trigger
   * a tooltip to show.
   */
  showTooltips(show: boolean): void;
  /**
   * FIXME: Move this to settings, this sets if we should show the tooltip icon, and not trigger
   * a tooltip to show.
   */
  toggleTooltips(): void;
  showErrorAlert: (message: string) => void;
}

export interface Tooltips {
  showTooltips: boolean;
}

export const TooltipsContext = React.createContext<Tooltips>({
  showTooltips: true,
});

export const AppControllerContext = React.createContext<AppController>({
  setShowSettings: () => {},
  showHelp: () => {},
  toggleHelp: () => {},
  showHotkeysDialog: () => {},
  toggleHotkeysDialog: () => {},
  showTooltips: () => {},
  toggleTooltips: () => {},
  showErrorAlert: () => {},
});

export const AppConfigContext = React.createContext<AppConfig>(appConfig);
