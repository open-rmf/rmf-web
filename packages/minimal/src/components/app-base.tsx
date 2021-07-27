import { Grid, makeStyles } from '@material-ui/core';
import React from 'react';
import dataConfig from '../config/data-config';
import { loadSettings, saveSettings } from '../settings';
import {
  AppController,
  AppControllerContext,
  DataConfigContext,
  SettingsContext,
} from './app-contexts';
import { AppDrawers } from './app-drawers';
import AppBar from './appbar';

const useStyles = makeStyles((theme) => ({
  appBase: {
    width: '100%',
    height: '100%',
    backgroundColor: theme.palette.background.default,
  },
}));

export interface AppBaseProps {}

/**
 * Contains various components that are essential to the app and provides contexts to control them.
 * Components include:
 * - Settings
 * - Notifications
 *
 * Also provides `AppControllerContext` to allow children components to control them.
 */
export function AppBase({ children }: React.PropsWithChildren<AppBaseProps>): JSX.Element | null {
  const classes = useStyles();

  const [settings, setSettings] = React.useState(() => loadSettings());
  const [showSettings, setShowSettings] = React.useState(false);

  const appController = React.useMemo<AppController>(
    () => ({
      showSettings: setShowSettings,
      setSettings,
      saveSettings,
      toggleSettings: () => setShowSettings((prev) => !prev),
    }),
    [],
  );

  return (
    <SettingsContext.Provider value={settings}>
      <AppControllerContext.Provider value={appController}>
        <DataConfigContext.Provider value={dataConfig}>
          <Grid container direction="column" className={classes.appBase} wrap="nowrap">
            <AppBar />
            {children}
            <AppDrawers settings={settings} showSettings={showSettings} />
          </Grid>
        </DataConfigContext.Provider>
      </AppControllerContext.Provider>
    </SettingsContext.Provider>
  );
}
