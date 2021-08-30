import React from 'react';
import appConfig, { AppConfig } from '../app-config';
import dataConfig, { DataConfig } from '../config/data-config';
import { ResourceManager } from 'rmf-tools';

/* Declares the ResourcesContext which contains the resources used on the app*/
export const ResourcesContext = React.createContext<ResourceManager | undefined>(undefined);

export const AppConfigContext = React.createContext<AppConfig>(appConfig);

export const DataConfigContext = React.createContext<DataConfig>(dataConfig);
