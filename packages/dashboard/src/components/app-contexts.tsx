import { AlertProps } from '@mui/material';
import React from 'react';
import appConfig, { AppConfig } from '../app-config';
import ResourceManager from '../managers/resource-manager';
import { defaultSettings, Settings } from '../settings';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager | undefined>(undefined);

export const SettingsContext = React.createContext(defaultSettings());

export interface AppController {
  updateSettings: (settings: Settings) => void;
  showAlert: (severity: AlertProps['severity'], message: string, autoHideDuration?: number) => void;
  setExtraAppbarIcons: (node: React.ReactNode) => void;
}

export interface Tooltips {
  showTooltips: boolean;
}

export const TooltipsContext = React.createContext<Tooltips>({
  showTooltips: true,
});

export const AppControllerContext = React.createContext<AppController>({
  updateSettings: () => {},
  showAlert: () => {},
  setExtraAppbarIcons: () => {},
});

export const AppConfigContext = React.createContext<AppConfig>(appConfig);
