import React from 'react';
import appConfig, { AppConfig } from '../app-config';
import dataConfig, { DataConfig } from '../config/data-config';
import ResourceManager from '../managers/resource-manager';
import { defaultSettings, Settings } from '../settings';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager | undefined>(undefined);

export const SettingsContext = React.createContext(defaultSettings());

export interface AppController {
  setSettings: React.Dispatch<React.SetStateAction<Settings>>;
  saveSettings(settings: Settings): void;
  showSettings(show: boolean): void;
  toggleSettings(): void;
}

export const AppControllerContext = React.createContext<AppController>({
  setSettings: () => {},
  saveSettings: () => {},
  showSettings: () => {},
  toggleSettings: () => {},
});

export const AppConfigContext = React.createContext<AppConfig>(appConfig);

export const DataConfigContext = React.createContext<DataConfig>(dataConfig);

export interface AppContent {
  tabNames: string[];
}

export const TrajectorySocketContext = React.createContext<WebSocket | undefined>(undefined);
