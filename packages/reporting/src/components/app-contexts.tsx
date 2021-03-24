import React from 'react';
import appConfig, { AppConfig } from '../app-config';

export const AppConfigContext = React.createContext<AppConfig>(appConfig);
