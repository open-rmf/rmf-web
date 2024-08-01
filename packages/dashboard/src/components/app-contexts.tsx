import { AlertProps } from '@mui/material';
import React from 'react';

import { defaultSettings, Settings } from '../settings';

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
