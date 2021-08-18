import React from 'react';
import appConfig, { AppConfig } from '../app-config';
import dataConfig, { DataConfig } from '../config/data-config';
import ResourceManager from '../managers/resource-manager';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager | undefined>(undefined);

export const AppConfigContext = React.createContext<AppConfig>(appConfig);

export const DataConfigContext = React.createContext<DataConfig>(dataConfig);

export interface AppContent {
  tabNames: string[];
}

export const TrajectorySocketContext = React.createContext<WebSocket | undefined>(undefined);
