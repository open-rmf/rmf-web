import React from 'react';
import appConfig, { AppConfig } from '../app-config';
// import { defaultSettings, Settings } from '../settings';

// export const SettingsContext = React.createContext(defaultSettings());

export const AppConfigContext = React.createContext<AppConfig>(appConfig);
